# State.h — line-by-line read

State.h is a header with no .cpp: it defines the fixed records that cross
module boundaries and the `ReferenceSource` contract every target provider
implements. Deliberately Eigen-only — no Kortex, no Pinocchio — so any
file can include it without dragging in the SDK or the model library
(that is also why the portable tests can build against it).

There is no "execution order" here in the usual sense; these types are
*used* in this order during a run:

1. `RobotState` — filled by the Runner from feedback every cycle.
2. `Reference` (+ `PoseReference` / `JointReference` / `Twist`) — produced
   by the source's `Get`, consumed by the controller, every cycle.
3. `ControllerStatus` — reset and refilled every cycle; the Runner copies
   it into the log row.
4. `ReferenceSource` — the interface: one `Reset` at takeover, one `Get`
   per cycle.

## The architecture comment (lines 5-16)

The ASCII diagram is the module map: sources (Targets.h, Trajectory.h,
future Vicon) produce a `Reference`; THE controller (Controller.h) turns it
into q̇; the Runner clamps and the actuation integrates. The rule in the
last line is the design's spine: **new research inputs are new sources,
never new controllers**. Worth keeping even after simplification — it is
why removing stdin input touches only Targets, not the controller.

## `RobotState` (lines 30-34)

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
- **The membership rule (comment, 28-29)**: a field belongs here only if
  the Runner can fill it validly EVERY cycle from arm feedback alone;
  external sensing must arrive as a `Reference` instead. This rule is what
  keeps "measured state" trustworthy — nothing in it can be stale or
  interpolated. **FLAG `State.h:28-34` | edit-hazard** — adding a
  convenient field here (say, a Vicon pose) would violate the invariant
  every consumer assumes; the compiler will not stop you, only this
  comment does.
- Note `q_rad`/`qdot_rad_s` are *uninitialized* by default (Eigen does not
  zero-fill). The Runner fills them before first use; constructing a
  `RobotState` and reading it without filling is garbage.

## `ControllerStatus` (lines 39-66)

Per-cycle telemetry, data only — the Runner decides what to print or log
(sources and controllers do no I/O). NaN is the "this cycle did not
compute it" sentinel. First-time note:
`std::numeric_limits<double>::quiet_NaN()` produces the floating-point
"not a number" value; every comparison with NaN is false, and the CSV
writers print it as `nan`, which the analysis scripts treat as "absent".

- `p_desired` / `p_current` (40-41) — the Cartesian target and FK position
  this cycle; both go straight into the log row.
- `arrived_edge` / `arrival_error_m` (42-43) — *edge-triggered*: true only
  on the first cycle a new target comes within tolerance, so the Runner
  prints the arrival notice exactly once. The controller owns the
  edge-detection; this is just the wire.
- `sigma_min` (44) — smallest singular value of the task Jacobian: the
  proximity-to-singularity number.
- `rot_error_rad` (45) — rotation-log error norm, NaN when the law has no
  orientation task.
- `tool_quat` (50-54) — the measured tool orientation as an
  `Eigen::Quaterniond`, Hamilton convention, hemisphere-fixed to w ≥ 0 so
  logged curves never flip sign. Initialized to four NaNs — note
  Eigen's quaternion constructor takes (w, x, y, z) but stores/prints
  coefficients as (x, y, z, w); the Runner's `FillSample` comment calls
  this out. **FLAG `State.h:50-54` | edit-hazard** — the w-first
  constructor vs x-first storage is a classic quaternion bug source.
- `q_ref_deg`, `playback_t_s`, `playback_state`, `playback_done_edge`,
  `playback_refused_edge` (56-65) — **FLAG `State.h:56-65` |
  trajectory-only** — filled only by `TrajectorySource`; the operator
  source leaves the NaN/0/false defaults. The two `*_edge` flags drive the
  Runner's one-shot playback prints, and `playback_state`'s encoding
  (0 none, 1 playing, 2 done, 3 refused) is written into every CSV row.

## `Twist` (lines 72-75)

Linear + angular velocity, both defaulting to zero — mirroring the Python
sim's `Twist.zero()`: a source that sets nothing commands a stationary
target. **FLAG `State.h:72-75` | unnecessary** *(today)* — every existing
source leaves it zero, so in the current program it is always pure-damping
plumbing; it exists as the seam for a future moving-target source (the
`PoseReference` comment explains a source that moves its target must fill
it or the Kd term fights the motion). Under a strict fixed-target-only
trim it could go, at the price of re-plumbing when a moving source
arrives.

## `PoseReference` (lines 87-92)

One Cartesian target: position, optional rotation, feed-forward twist, and
a `sequence`.

- `rotation` is `std::optional<Eigen::Matrix3d>` — first-time note:
  `std::optional<T>` holds either a `T` or nothing; here "nothing"
  (`std::nullopt`) *means something*: "no orientation requested — keep the
  takeover hold orientation". Encoding "keep" as absence instead of a
  magic value is the pattern this codebase prefers.
- `sequence` identifies distinct targets so the arrival notice fires once
  per target (see Targets.md for the sequence-0 fixed-target convention).

## `JointReference` (lines 97-100)

**FLAG `State.h:94-100` | trajectory-only** — where the joints should be
now *and at t + dt*. Only `TrajectorySource` produces one. The pair
encodes the feed-forward velocity exactly — the integrator "telescopes"
(each cycle's step ends exactly where the next begins) with zero
discretization drift, which a (position, velocity) pair would not
guarantee under a clamped dt.

## `Reference` (lines 107-110)

```cpp
std::optional<PoseReference> pose;
std::optional<JointReference> joints;
```

The per-cycle hand-off from source to controller: at most one channel set.
The comment carries two rules a future edit must not break:

- **Joints win when both are set** — a planner's joint path must not be
  re-solved through the pose law, or damped least squares could pick
  different joint motions and undo the planner's collision avoidance.
  (The priority itself is implemented in the controller; this comment is
  its specification.)
- **Neither set = "no reference"** — the controller holds the takeover
  pose. This is why an idle operator run simply holds still.

**FLAG `State.h:102-110` | edit-hazard** — the "at most one channel"
invariant is enforced by convention, not by the type (both optionals could
be set); a source that sets both would silently have its pose ignored.
*(The `joints` channel is also trajectory-only in practice — covered by
the JointReference flag above.)*

## `ReferenceSource` (lines 114-127)

The abstract interface every source implements. First-time C++ notes:

- `virtual ... = 0` declares a *pure virtual* function — the class is
  abstract (cannot be instantiated) and every concrete source must
  override the method. Calls through a `ReferenceSource&` dispatch at
  runtime to the actual source's override — that is how the Runner runs
  either the operator source or the trajectory source without knowing
  which.
- `virtual ~ReferenceSource() = default;` (117) — a virtual destructor is
  required because Main holds sources in a
  `std::unique_ptr<ReferenceSource>`: deleting through the base pointer
  without it would destroy only the base part (undefined behaviour).

The contract in the comments is the load-bearing part:

- **Line 112**: "Pure computation: no I/O, no allocation, no blocking
  beyond a bounded store lock" — `Get` runs inside the 2 ms cycle;
  a source that reads a file or sleeps in `Get` stalls the arm's command
  stream.
- `Reset` (119-121) — called once at T5 of takeover, after
  `PositionIntegration::Prepare`, before the first `Get`; captures the
  source's baseline (store sequence for the operator source; the
  start-state re-gate for the trajectory source).
- `Get` (123-126) — this cycle's reference; may fill only the
  `ControllerStatus` fields that describe the reference itself.

**FLAG `State.h:114-127` | edit-hazard** — the Reset-before-first-Get
ordering and the "no blocking in Get" rule are enforced nowhere in code;
they live in these comments and in the Runner's call order. A new source
that does I/O in `Get` compiles cleanly and misbehaves only on hardware.
