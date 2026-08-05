# Controller.cpp + Controller.h — line-by-line read

**Entry points:** Main.cpp:323 constructs `TrackingController controller(controlled_model);`
before the takeover (so a bad frame name fails early). After that, only the
Runner calls it: `controller.Reset(state)` once at takeover T4/T5 seeding
(Runner.cpp:163), then `controller.DesiredVelocity(state, reference, dt_s,
status)` once per cycle (Runner.cpp:215–217). The destructor runs at Main's
scope exit.

The design rule this file enforces: there is ONE controller. Reference
sources differ per run (today only `PoseTargetSource` — operator targets or
the compiled fixed target; the trajectory source was removed with playback);
the controller tracks the pose the source filled, and a silent source means
"hold the takeover pose" — so a source that produces nothing can never cause
motion.

---

## Controller.h

### Lines 1–16 — the banner
The routing table: pose channel → reactive law (ReactiveLaw.h), no
reference → reactive law toward the pose captured at `Reset`. It also
declares what this file must NOT contain: no frame conversion, integration,
timing, hardware access, printing, or allocation. That is a real contract —
`DesiredVelocity` runs inside the 2 ms cycle.

### Lines 18–29 — includes and forward declarations
Includes are Eigen plus the two shared-vocabulary headers (ReactiveLaw.h,
State.h). Lines 28–29 are *forward declarations*: `class DualArmKinematics;`
tells the compiler "this type exists" without defining it, which is enough
to hold a reference or a `unique_ptr` to it. The point (stated at line 15)
is to keep Pinocchio — a heavy dependency — out of every file that includes
this header; only Controller.cpp sees the real definitions.

### Lines 31–50 — the public interface
- **38** — `explicit TrackingController(DualArmKinematics& model);`
  `explicit` forbids the compiler from silently converting a
  `DualArmKinematics` into a controller in places expecting one — a
  single-argument constructor without it creates that conversion. The model
  comes in by reference: borrowed, owned by Main.
- **39** — a declared destructor. Non-obvious why until you see line 54: the
  class holds a `std::unique_ptr<KinematicsWorkspace>` to a type this header
  only forward-declares. `unique_ptr`'s destructor must call the pointee's
  destructor, which needs the full type — so the destructor is *declared*
  here and *defined* in the .cpp (line 42, `= default`), where the type is
  complete. Delete this line and the header stops compiling for every
  includer, with a confusing "incomplete type" error.
- **43** — `Reset`: captures the CURRENT end-effector pose as the hold pose
  and disarms the arrival notice. Called exactly once per run, at takeover.
- **47–50** — `DesiredVelocity`: the per-cycle call. Returns rad/s BEFORE
  clamping (the Runner clamps, the actuation integrates — never this class).
  `status` is an out-parameter for telemetry.

### Lines 52–71 — private state
- **53–54** — the model reference and the owned workspace.
  `std::unique_ptr<T>` is a smart pointer that owns one heap object and
  deletes it automatically when the pointer dies — no manual `delete`, and
  it cannot be copied (ownership is unique). The workspace is Pinocchio's
  scratch memory, allocated once so the cycle allocates nothing.
- **58–60** — gains and the null-space targets, converted from Config once
  in the constructor "because the loop must not repeat it" — the deg→rad
  conversion of seven constants is trivially cheap, but the rule "no
  repeated setup work in the cycle" is applied uniformly.
- **64–65** — the hold pose: position and rotation captured at `Reset`.
  Zero/Identity defaults are placeholders only; `Reset` always runs before
  the first `DesiredVelocity` (the Runner's T4 ordering guarantees it).
  **[edit-hazard, lines 64–65]** If anything ever called `DesiredVelocity`
  before `Reset`, an empty reference would command the arm toward position
  (0,0,0) — inside the robot's own base. The safety here is purely the
  Runner's calling order, not this class.
- **69–71** — the arrival-notice state machine: `last_pose_sequence_` (which
  target we last saw), `pose_sequence_seen_` (have we seen any), and
  `arrival_reported_` defaulting to **true**. **[edit-hazard, lines 69–71]**
  That `true` default is the reason the hold pose never fires an arrival
  notice: the notice only arms (goes false) when a *new pose sequence*
  arrives. Flip the default to false and every run prints a spurious
  "target reached" at takeover.

---

## Controller.cpp

### Lines 6–9 — includes
`Controller.h` first, then Config (gains) and Kinematics (the real
`DualArmKinematics` / `KinematicsWorkspace` definitions — this is the one
translation unit where the controller meets Pinocchio).

### Lines 11–29 — `ConfiguredGains()` (runs once, from the constructor)
Builds a `ReactivePoseGains` struct (ReactiveLaw.h:38) from Config
constants: Kp position/rotation, Kd position/rotation, the limit-avoidance
gain (`kLimitAvoidGain`), DLS λ, and the three term switches. **[edit-hazard, lines 15–28]** Note what is *not*
set: `position_enabled` stays at its struct default (`true`). There is no
`kPositionEnabled` in Config.h even though the gains struct has the switch —
someone adding one must remember this function, or the new config constant
will compile and do nothing.

### Lines 31–40 — the constructor
Runs in Main, before any hardware connection. The member-initializer list
(`: model_(model), workspace_(...), gains_(...)`) initializes members in
declaration order: bind the model reference, heap-allocate the Pinocchio
workspace once (`std::make_unique` builds the object and wraps it in the
`unique_ptr` in one step), copy the gains. The body converts
`kJointSoftwareLimitDeg` deg→rad into `limit_rad_` (per joint; 0 stays 0,
the sentinel for an unbounded joint) and `kLimitAvoidZoneDeg` deg→rad into
the single `zone_rad_`, so `LimitAvoidanceVelocity` never touches Config.h
units directly.

### Lines 44–54 — `Reset` (takeover)
Runs forward kinematics on the takeover-seeded measurement
(`RightPoseAndJacobian` composes the 14-joint configuration from the
measured right arm plus the fixed left nominal, and returns the right arm's
pose and 6×7 Jacobian). The resulting position and rotation become the hold
pose — "an empty reference resolves to the pose measured at takeover".
`pose_sequence_seen_ = false; arrival_reported_ = true;` re-arm the
sequence tracker and disarm the notice. If `Reset` were skipped, the hold
pose would be stale (or zero — see the header hazard above).

### Lines 56–128 — `DesiredVelocity` (once per cycle)
Follow it in execution order:

- **61–67 — resolve the pose target.** `Reference` (State.h:84) now carries
  only an optional pose. `std::optional<T>` is a container holding either
  one value or nothing; in a boolean context it means "is a value
  present?". Three-level fallback: `reference.pose->p_desired` if the
  source gave a pose, else the hold position; `*reference.pose->rotation`
  if the pose *also* carries an orientation (`rotation` is itself an
  optional inside the optional — "position-only target" is a real case),
  else the hold rotation. This is the line pair that makes "silent source =
  hold here" true.

- **69–78 — arrival-notice arming.** If a pose is present and its
  `sequence` number differs from the last one seen (or none was seen yet),
  record it and set `arrival_reported_ = false` — the notice is now armed
  for this target. Each typed target gets exactly one "target reached"
  print; the hold pose (no `reference.pose` at all) can never arm it.

- **80–83 — FK + Jacobian.** The one model call of the cycle, into the
  preallocated workspace. Same adapter as `Reset`: measured right joints +
  fixed left nominal, right 7 columns selected.

- **85–88 — Equation 1, pose error.** `e_pos = p_desired − p_actual`, and
  `e_rot = RotationLog(R_des · Rᵀ)` — the rotation "difference" mapped to an
  angle·axis 3-vector (ReactiveLaw.md explains why this never wraps the long
  way).

- **90–97 — Equation 2, twist error.** The measured end-effector twist is
  `J·q̇_measured` (6-vector: linear then angular). The reference twist is the
  source's feed-forward velocity, or a default-zero `Twist{}` when holding —
  the comment's point: the hold pose is genuinely stationary, so zero is its
  true velocity, not a placeholder. `TwistError` is reference − measured.

- **99–105 — the arrival check.** If armed and the position error norm
  drops under `kArrivalToleranceM` (1 mm), disarm and set the edge flag +
  error into `status`. The controller sets data; the Runner prints — the
  "controllers do no I/O" rule again.

- **106–111 — telemetry fill.** Desired/current position, rotation-error
  norm, and the measured tool quaternion. Lines 110–111 flip the whole
  quaternion's sign when w < 0. **[hides-work, lines 109–111]** q and −q are
  the *same rotation* (quaternion double cover), so this changes nothing
  physically — it pins the log to the w ≥ 0 hemisphere so plotted quaternion
  components never jump sign mid-run. It looks like it might be a correction;
  it is purely a logging convention (State.h documents it).

- **113–118 — σ_min.** Builds `J·Jᵀ` (6×6), eigendecomposes it
  (`SelfAdjointEigenSolver` — the solver for symmetric matrices; its
  eigenvalues come out sorted ascending, so index 0 is the smallest), and
  takes √max(0, λ_min) = the smallest singular value of the Jacobian: the
  "how close to a singularity" health number. The `max(0, ·)` guards against
  a tiny negative eigenvalue from floating-point noise reaching `sqrt`.
  **[mixed-jobs, lines 113–118]** This is pure diagnostics — nothing in the
  control law uses σ_min — computed inside the control function, every
  cycle, including a full 6×6 eigensolve. It belongs with the law's inputs
  only in the sense that the Jacobian is already at hand; it is logging work
  living in the controller. (Cost is small and fixed-size, so it is a
  tidiness flag, not a performance one. Note the cycle now forms `J·Jᵀ`
  three times: here, in `DampedLeastSquares6`, and in
  `LimitAvoidanceVelocity`.)

- **144–152 — Equations 3–6.** Copy the gains, scale only the
  limit-avoidance gain by `UnitRamp(t_s, kNullRampDurationS)` — the
  limit-avoidance push fades in over the first second after takeover so the
  takeover cannot begin with a projected joint transient; the task-space law
  is at full strength immediately. Then
  hand everything to `SolveReactiveVelocityDetailed` (ReactiveLaw.md), which
  returns the raw desired q̇ (task + limit-avoidance) the Runner will clamp.
  The per-cycle copy of the whole
  gains struct to scale one field is slightly wasteful but keeps `gains_`
  immutable — a reasonable trade.
