# State.h — line-by-line read

*(Updated for commit f64325c0: `JointReference`, the playback telemetry in
`ControllerStatus`, and the `joints` channel of `Reference` were removed
with trajectory playback. The file is now 103 lines.)*

State.h is a header with no .cpp: it defines the fixed records that cross
module boundaries and the `ReferenceSource` contract every target provider
implements. Deliberately Eigen-only — no Kortex, no Pinocchio — so any
file can include it without dragging in the SDK or the model library
(that is also why the portable tests can build against it).

There is no "execution order" here in the usual sense; these types are
*used* in this order during a run:

1. `RobotState` — filled by the Runner from feedback every cycle.
2. `Reference` (+ `PoseReference` / `Twist`) — produced by the source's
   `Get`, consumed by the controller, every cycle.
3. `ControllerStatus` — reset and refilled every cycle; the Runner copies
   it into the log row.
4. `ReferenceSource` — the interface: one `Reset` at takeover, one `Get`
   per cycle.

## The architecture comment (lines 5-15)

The ASCII diagram is the module map: sources (Targets.h today, future
Vicon) produce a `Reference`; THE controller (Controller.h) turns it into
q̇; the Runner clamps and the actuation integrates. The rule in the last
line is the design's spine: **new research inputs are new sources, never
new controllers**. It just proved itself: removing trajectory playback
deleted a source and its channel, and the controller survived with only
the joint-reference branch excised.

## `RobotState` (lines 29-33)

```cpp
Eigen::Matrix<double, 7, 1> q_rad;
Eigen::Matrix<double, 7, 1> qdot_rad_s;
double t_s = 0.0;
```

- **What**: measured joint positions/velocities (radians — the boundary
  conversion from Kortex degrees happens in the Runner) and time since
  takeover. `Eigen::Matrix<double,7,1>` is a fixed-size column vector:
  size known at compile time, so no heap allocation, and a size mismatch
  is a compile error.
- **The membership rule (comment, 26-28)**: a field belongs here only if
  the Runner can fill it validly EVERY cycle from arm feedback alone;
  external sensing must arrive as a `Reference` instead. This rule is what
  keeps "measured state" trustworthy — nothing in it can be stale or
  interpolated. **FLAG `State.h:26-33` | edit-hazard** — adding a
  convenient field here (say, a Vicon pose) would violate the invariant
  every consumer assumes; the compiler will not stop you, only this
  comment does.
- Note `q_rad`/`qdot_rad_s` are *uninitialized* by default (Eigen does not
  zero-fill). The Runner fills them before first use; constructing a
  `RobotState` and reading it without filling is garbage.

## `ControllerStatus` (lines 38-54)

Per-cycle telemetry the controller surfaces, data only — the Runner
decides what to print or log (sources and controllers do no I/O). NaN is
the "this cycle did not compute it" sentinel. First-time note:
`std::numeric_limits<double>::quiet_NaN()` produces the floating-point
"not a number" value; every comparison with NaN is false, and the CSV
writers print it as `nan`, which the analysis scripts treat as "absent".

- `p_desired` / `p_current` (39-40) — the Cartesian target and FK position
  this cycle; both go straight into the log row.
- `arrived_edge` / `arrival_error_m` (41-42) — *edge-triggered*: true only
  on the first cycle a new target comes within tolerance, so the Runner
  prints the arrival notice exactly once. The controller owns the
  edge-detection; this is just the wire.
- `sigma_min` (43) — smallest singular value of the task Jacobian: the
  proximity-to-singularity number.
- `rot_error_rad` (44) — rotation-log error norm, NaN when the law has no
  orientation task.
- `tool_quat` (49-53) — the measured tool orientation as an
  `Eigen::Quaterniond`, Hamilton convention, hemisphere-fixed to w ≥ 0 so
  logged curves never flip sign. Initialized to four NaNs — note
  Eigen's quaternion constructor takes (w, x, y, z) but stores/prints
  coefficients as (x, y, z, w); the Runner's `FillSample` comment calls
  this out. **FLAG `State.h:49-53` | edit-hazard** — the w-first
  constructor vs x-first storage is a classic quaternion bug source.

(The playback telemetry that used to end this struct — `q_ref_deg`,
`playback_t_s`, `playback_state`, the two playback edges — was removed
with the trajectory source; the CSV lost the matching nine columns,
log_format 5 → 6.)

## `Twist` (lines 60-63)

Linear + angular velocity, both defaulting to zero — mirroring the Python
sim's `Twist.zero()`: a source that sets nothing commands a stationary
target. **FLAG `State.h:60-63` | unnecessary** *(today)* — the program's
one source leaves it zero, so it is always pure-damping plumbing; it
exists as the seam for a future moving-target source (the `PoseReference`
comment explains a source that moves its target must fill it, or the Kd
term fights the motion). A stricter trim could remove it, at the price of
re-plumbing when a moving source arrives.

## `PoseReference` (lines 75-80)

One Cartesian target: position, optional rotation, feed-forward twist, and
a `sequence`.

- `rotation` is `std::optional<Eigen::Matrix3d>` — first-time note:
  `std::optional<T>` holds either a `T` or nothing (`std::nullopt`); here
  "nothing" *means something*: "no orientation requested — keep the
  takeover hold orientation". Encoding "keep" as absence instead of a
  magic value is the pattern this codebase prefers.
- `sequence` identifies distinct targets so the arrival notice fires once
  per target. The fixed-target source always sends sequence 0 — one
  target, one arrival notice (see Targets.md).
- The comment block (65-74) is the `twist` contract described under
  `Twist` above.

## `Reference` (lines 82-86)

```cpp
struct Reference {
    std::optional<PoseReference> pose;
};
```

The per-cycle hand-off from source to controller, now a single optional
channel: set means "track this pose", unset means "no reference — hold the
takeover pose" (comment, 82-83). This is why a run with no target simply
holds still. The old second channel (`joints`, the planner's exact joint
path) and its joints-win-over-pose priority rule went with trajectory
playback; if a joint-space source ever returns, this struct is where its
channel — and the priority comment — would be reintroduced.

## `ReferenceSource` (lines 90-103)

The abstract interface every source implements. First-time C++ notes:

- `virtual ... = 0` declares a *pure virtual* function — the class is
  abstract (cannot be instantiated) and every concrete source must
  override the method. Calls through a `ReferenceSource&` dispatch at
  runtime to the actual source's override — that is how the Runner drives
  whatever source Main constructed without knowing its type.
- `virtual ~ReferenceSource() = default;` (93) — a virtual destructor is
  required because Main holds the source in a
  `std::unique_ptr<ReferenceSource>`: deleting through the base pointer
  without it would destroy only the base part (undefined behaviour).

The contract in the comments is the load-bearing part:

- **Lines 88-89**: "One Reset at takeover, then one Get per cycle. Pure
  computation: no I/O, no allocation, no blocking" — `Get` runs inside the
  2 ms cycle; a source that reads a file or sleeps in `Get` stalls the
  arm's command stream.
- `Reset` (95-97) — called once at T5 of takeover, after
  `PositionIntegration::Prepare`, before the first `Get`; captures the
  source's baseline (nothing, for the fixed target — its override is
  empty; a future sensing source would latch its starting state here).
- `Get` (99-102) — this cycle's reference; may fill only the
  `ControllerStatus` fields that describe the reference itself.

**FLAG `State.h:90-103` | edit-hazard** — the Reset-before-first-Get
ordering and the "no blocking in Get" rule are enforced nowhere in code;
they live in these comments and in the Runner's call order
(Runner.cpp:164 then 214). A new source that does I/O in `Get` compiles
cleanly and misbehaves only on hardware.
