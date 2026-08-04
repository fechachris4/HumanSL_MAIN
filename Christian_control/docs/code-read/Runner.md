# Runner.cpp + Runner.h — line-by-line read

**Entry point:** `Main.cpp:400` calls `RunControlLoop(...)` exactly once, after
the readiness gate, the CSV setup and the reference-source construction. This
is the function that MOVES THE ARM. Everything in this file exists to run the
takeover sequence T1–T6 (documented in the Runner.h banner, lines 6–24), the
500 Hz cycle, and the teardown D1–D2 on every possible exit path.

This doc follows real execution order: the header contract first, then
`RunControlLoop` top to bottom (which is also its execution order), with the
helper `FillSample` explained where the loop first calls it.

---

## Runner.h

### Lines 1–34 — the banner comment
Plain-English spec of the whole file: the takeover order (T1–T6), the per-cycle
order, and the teardown order (D1–D2). Not code, but treat it as normative —
the .cpp is written to match it step for step, and several past bugs were
order bugs. If you edit the loop, update this comment or the two will drift.

### Lines 36–51 — `#pragma once` and includes
`#pragma once` tells the compiler "include this header at most once per
translation unit" (the modern replacement for include guards). The includes
pull in the standard clamp/atomic/chrono utilities, the two Kortex client
headers (the robot API), and the five sibling modules the loop composes:
Actuation (integrator), Config (constants), Controller (the law), Hardware
(cyclic exchange + log), Safety (guards + reports), State (shared structs).

### Lines 52–62 — the `RunControlLoop` declaration
One free function, no class. Parameter by parameter:

- `base` / `base_cyclic` — raw pointers to the two Kortex clients owned by
  `Connect` in Main. Raw pointers here mean "borrowed, not owned": the Runner
  uses them but Main's `Connect` object destroys them. If Main ever let
  `Connect` die before this call, these would dangle — the current call order
  makes that impossible.
- `ReferenceSource& reference` — a C++ *reference* (an alias for an existing
  object, never null). `ReferenceSource` is an abstract base class
  (State.h:90); since the trajectory-playback removal its only implementation
  is `PoseTargetSource` (typed operator targets, or the compiled fixed
  target). The Runner never knows which mode — that plug-in seam is kept so a
  future source (e.g. Vicon) slots in without touching the loop.
- `TrackingController& controller` — THE controller (Controller.md).
- `PositionIntegration& actuation` — the velocity→position integrator.
- `LoopLog& log` — the lock-free handoff queue to the CSV writer thread.
- `const std::atomic<bool>& stop` — Main's `g_stop`, set by the Ctrl+C signal
  handler. `std::atomic<bool>` is a bool that can be safely read and written
  from different threads (and from a signal handler) at the same time without
  a mutex; a plain `bool` shared like that is undefined behaviour.
- `period` — 2 ms (`config::kCyclePeriod`), the loop grid.
- `qdot_limit_deg_s` — the per-joint speed clip, the program's single speed
  limit.
- `following_error_limit_deg` — 3.0°, the tracking guard threshold.
- `robot_ready` — the T1 gate result, asserted below.

Returns `LoopResult` (Safety.h:61): why the loop stopped, whether any live
fault was seen, and the final time/cycle count.

---

## Runner.cpp

### Lines 5–16 — includes and the namespace alias
Standard headers plus `KDetailedException.h`, the Kortex exception type the
catch ladder needs, then `Runner.h` itself. `namespace k_api = Kinova::Api;`
is a namespace *alias* — a short local name for a long namespace, purely
cosmetic.

### Lines 18–22 — file-local constants
The anonymous `namespace { ... }` makes everything inside it private to this
.cpp file (like `static` for free functions). `NUM_JOINTS` is derived from
`JointVector` (`std::array<double, 7>`) via `std::tuple_size_v`, so "7" is
written once, in Config.h. The two conversion constants exist because the
robot speaks degrees and the math speaks radians; this file is the boundary
where conversions happen.

### Lines 24–70 — `FillSample` (called once per cycle, line 272)
Copies one cycle's worth of data into a `LoopLogSample` (the CSV row,
Hardware.h:256, log_format 6). Field groups:

- **34–38** — desired and current end-effector position from
  `ControllerStatus` (what the controller computed this cycle).
- **39–42** — `sigma_min` (Jacobian conditioning), rotation error, and the
  measured tool quaternion. `status.tool_quat.coeffs()` is Eigen's internal
  x,y,z,w storage order — the comment is there because it is the opposite of
  the w-first constructor order and a classic source of silent bugs.
- **43–49** — copies controller status into the log sample. This helper does
  not perform target admission or reachability checks; those concerns are not
  represented in the runtime telemetry.
- **50–64** — per-joint fields. Line 55–56 is the important one:
  `measured_deg = commanded + remainder(raw − commanded, 360)`.
  `std::remainder(x, 360)` returns x reduced to the range (−180, +180], so
  this shifts the robot's wrapped [0, 360) feedback by whole turns until it
  sits within ±180° of the continuous integrated command. Command and
  measurement then live on the same axis, and `cmd − meas` is a real tracking
  error. **[edit-hazard, lines 55–56]** The following-error guard in
  Safety.cpp compares exactly these two columns — if this wrap ever changes,
  the guard silently changes meaning with it, because the guard itself does
  no wrapping. `measured_raw_deg` (line 57) keeps the untouched value so the
  file is never ambiguous.
- **65–69** — frame IDs, arm state, base fault bank, `refresh_ok = true`
  (the exchange succeeded; the catch blocks set it false on the failure row).

### Lines 73–87 — entry and the T1 assertion
The signature repeats the header. Lines 85–87:

```cpp
assert(robot_ready);
if (!robot_ready)
    throw std::logic_error(...);
```

`assert` aborts the program in debug builds if the condition is false, and
compiles to *nothing* in release builds (when `NDEBUG` is defined) — which is
exactly why the `throw` follows it: the throw is the check that always runs.
**[unnecessary, line 85]** Given the unconditional throw on the next line,
the assert adds nothing except a louder crash in debug builds; it is
harmless, but the real guard is the throw (and, one level up, Main returning
before ever calling this function when the gate fails).

### Lines 89–105 — the compile-time stop-policy warnings
Three `std::cout` warnings, printed once before the loop, driven by
`config::kStopOnFault` and `config::kDisableFollowingErrorStop`. These are
`constexpr` values, so the compiler already knows which warnings print — the
point is that the *operator* knows too, at the terminal, before the arm
moves. The third (lines 102–105) fires only when both guards are off:
"YOU are the safety system." Nothing here changes behaviour; behaviour was
decided at compile time in Config.h.

### Lines 107–115 — loop-local state
- `nominal_dt_s` — 0.002 s, from the period.
- `reason = LoopStop::kUserStop` — the default outcome. Every stop path
  overwrites it; if nothing does, the loop ended because `stop` went true.
- `faults_observed` — sticky flag: any live fault ever seen taints the exit
  code even when the fault-ignoring policy keeps the loop running.
- `sample` — ONE `LoopLogSample`, reused every cycle. Reuse matters: the
  struct is big, and allocating per cycle inside a 2 ms loop is forbidden.
  **[edit-hazard, line 111]** Because it is reused, a field a cycle does not
  overwrite carries the *previous* cycle's value — the catch blocks (351–384)
  rely on this deliberately (they push the last partially-updated sample as
  evidence), but a new field added to `FillSample` that is only conditionally
  written would silently smear across cycles.
- `cycle`, `joint_fault_was_latched`, `counters` (the decision-12
  consecutive-cycle counters), `freshness_monitor` (ack-ID telemetry,
  Safety.md).

### Line 117 — `CyclicSession cyclic(base_cyclic);`
Constructs the object that owns the cyclic command frame (Hardware.h:146).
Construction is cheap and does not touch the network.

### Line 120 — T2: `ServoingGuard servoing_guard(base);`
**This line changes the robot's mode.** Constructing the guard sends
`SetServoingMode(LOW_LEVEL_SERVOING)` — from here until restore, this
process is the arm's controller and must stream commands. It is an RAII
object: *Resource Acquisition Is Initialization*, the C++ pattern where a
constructor acquires something and the destructor releases it, so the release
happens automatically on every way out of the scope, including exceptions.
**[edit-hazard, lines 117–120]** Order matters twice over: the guard must be
constructed *after* `cyclic` exists but *before* the `try` block (so the
destructor still restores single-level if anything below throws), and `Seed`
must come after it (commanding before the mode switch fails with
WRONG_SERVOING_MODE — that is the T2→T3 ordering in the banner).

### Lines 122–126 — the `try` and T3: `cyclic.Seed()`
Everything that can throw during the run is inside this one `try`. `Seed()`
is the program's single standalone feedback read inside the command loop.
**[hides-work, line 126]** `Seed` does more than read: it also initializes
the command frame's seven actuator slots (Hardware.h:138–145), and its
network round trip doubles as the settling time for the mode switch — the
comment says so, but the call looks like a plain read.

### Lines 128–133 — first `RobotState`
Copies the seed feedback into `state` (positions and velocities,
degrees→radians at this boundary). `RobotState` (State.h:29) is the small
Eigen struct every module downstream consumes.

### Lines 135–142 — fault-print seeding
`prev_joint_banks` / `prev_base_bank` record the fault banks *as they were at
entry*, so a bank already latched (which `RobotReadyForTakeover` tolerates
for the stale JOINT_FAULT summary) does not print as a fresh event on cycle
1. `fault_prints` caps the total number of fault-change prints (Safety.h:150).

### Lines 144–250 — T4/T5: streamed hold and the seeding hand-off
This block is the heart of the takeover and the most order-sensitive code in
the file. **[edit-hazard]**

- **144–147** — `commanded_deg` starts as the measured position (a "hold
  here" command). `commanded_velocity_deg_s{}` — the `{}` zero-initializes
  the array; without it a `std::array` of doubles holds garbage.
- **T4** — the unchanged command is sent for
  `config::kTakeoverHoldCycles` frames (250 at the compiled 2 ms period),
  so it occupies 0.5 s on the same `sleep_until` grid as normal control.
  Every completed reply is logged and resolves following error, low-level
  loss, live fault policy, and stale acknowledgement with the normal
  precedence. There is no joint-boundary proposal during this fixed hold.
- **T5** — the final hold reply reseeds `state` AND `commanded_deg`, so
  controller state starts from the last measured pose rather than the Seed
  pose.
- **T5** — the one-time seeding chain, in this order:
  `actuation.Prepare(state)` (integrator command = measurement — the ONLY
  time command state comes from measurement), then `controller.Reset(state)`
  (captures the hold pose). Reordering these changes what each one captures.
  The Actuation.h and State.h contracts both name this order.
- **T5** — one more holding frame; its reply becomes cycle 1's input, so
  the first control cycle starts from feedback that postdates all seeding.

### Lines 167–174 — clocks and status
`std::chrono::steady_clock` is the monotonic clock (never jumps when the
wall clock changes — mandatory for control timing). `log_start` anchors all
CSV timestamps across the takeover hold, while `control_start` is reset
after that hold so the controller's `RobotState::t_s` begins at zero;
`next_cycle` is the pacing grid; `status` is the
`ControllerStatus` telemetry struct, declared once outside the loop.

### Line 176 — `while (!stop)`
Reads the atomic every cycle. Ctrl+C (or SIGTERM) in Main sets it; the loop
notices within one cycle (≤ 2 ms) and exits with `reason` still at
`kUserStop` — the only successful outcome.

### Line 178 — `next_cycle += period;`
Advances the grid *first*, so pacing is "sleep until the next grid point",
not "sleep for a period after finishing" — errors do not accumulate.

### Lines 180–198 — dt measurement and overrun counting
- **183–189** — `dt_s` is the *measured* time since the previous cycle
  start, except cycle 0 uses the nominal 2 ms (there is no previous cycle),
  and it is clamped by `ClampedCycleDt` (Actuation.h:26) to at most 2×
  nominal — a scheduler stall must not integrate into one big position jump,
  which the base would fault on. dt is an input to the controller and the
  integrator, so it is sampled once, here, at cycle start.
- **190–198** — separately, the *unclamped* elapsed time is compared against
  `kOverrunFactor × nominal` (1.5× = 3 ms): a slow cycle increments both the
  consecutive counter (`counters.overrun`, reset to 0 by any healthy cycle)
  and the whole-run tally. Note the duplication: `t_now − t_prev` is computed
  twice (188 and 191) — same value, deliberate clarity over micro-saving.

### Lines 200–207 — refresh `state` from the previous exchange
The feedback consumed here was *received at the end of the previous cycle*
(or the post-reseed T5 holding frame, on cycle 1). Degrees→radians again at
the boundary, and `state.t_s` becomes seconds since `control_start` (not the
CSV's takeover-relative `log_start`). **[edit-hazard, lines 200–207]**
This one-cycle offset is a design fact the whole log annotates (Hardware.h
"cross-exchange row semantics", Hardware.h:250): in row i, the controller
inputs come from row i−1's reply. Moving the `Send` earlier or refreshing
`state` later changes the loop's phase and every offline analysis assumption.

### Lines 209–217 — reference, then controller
- **212** — `status = ControllerStatus{};` resets the telemetry struct to
  its defaults every cycle. **[edit-hazard, line 212]** The edge flag
  (`arrived_edge`) is only ever *set*, never cleared, by the controller —
  this reset is what makes it an edge. Remove it and the arrival notice
  prints every cycle forever.
- **213–214** — `reference.Get(state, dt_s, status)`: the source says WHERE
  to be this cycle (a pose, or nothing = hold).
- **215–217** — `controller.DesiredVelocity(...)`: the law turns that into a
  desired joint velocity, rad/s, BEFORE any clamping (Controller.md).

### Lines 219–225 — capture the requested velocity for the record
Copies the controller's raw output into `requested_velocity_deg_s` *before*
the non-finite hold and the clamp can touch it — so the CSV's `reqvel_j*`
columns are the controller's true output, including a NaN if it produced one
(that NaN in the log is the evidence behind a `kNonFiniteCommand` stop).

### Lines 227–235 — the non-finite hold
`allFinite()` is Eigen's "no NaN, no infinity anywhere" check. A non-finite
velocity is replaced with zero (this cycle holds position) and the
consecutive counter increments; a finite cycle resets it. The bad value is
never integrated — the integrator would be poisoned permanently by one NaN.

### Lines 237–243 — the arrival notice
Prints *inside the cycle*, but only on the edge cycle for each new target,
so the cost is bounded and rare. The controller never prints — it sets the
flag, the Runner prints (the "controllers do no I/O" rule).
**[edit-hazard, lines 237–243]** Any print added here that is *not*
edge-triggered runs blocking terminal I/O at 500 Hz and will cause overruns.

### Lines 245–257 — clamp, then integrate
- **249–254** — the per-joint speed clamp, the program's single speed limit:
  each velocity converted rad→deg, clamped to ±`qdot_limit_deg_s[i]` (45),
  converted back. **[edit-hazard, lines 249–254]** The double unit
  conversion is easy to break when editing — the limit array is in deg/s but
  the vector is rad/s; get one conversion wrong and the limit is off by a
  factor of 57. A clamp pinned at its limit is *allowed indefinitely*: far
  targets transit at clip speed by design (the comment records that the old
  saturation stop was removed).
- **255–257** — `actuation.Apply(...)` uses the already-clamped qdot to
  integrate `q_command += q̇·dt` into a proposed command. It then projects
  excessive lead to a candidate exactly 1° from the wrapped measurement and
  limits the final sent delta to `abs(qdot_clamped * dt)`. The final envelope wins on a
  discontinuous feedback step, so recovery can temporarily remain above 1°;
  the unchanged 3° following-error stop is the backstop. The call writes the
  result into `commanded_deg` / `commanded_velocity_deg_s` **through the
  reference parameters** — **[hides-work, lines 255–257]** the call
  *mutates* `commanded_deg` in place; nothing at the call site says "output
  parameter" except the Actuation.h signature. The returned `ApplyStatus`
  carries the pre-constraint requested setpoint and whether the lead
  projection was active. If its joint-warning field is set, the two output
  arrays instead hold the prior full command frame and zero velocities;
  `req_j - cmd_j` preserves the blocked proposal for the log.

### Lines 259–265 — the one exchange
`cyclic.Send(commanded_deg)` writes the setpoints into the command frame,
stamps frame/command IDs, sends over UDP, and returns this cycle's feedback —
which the *next* iteration consumes (the phase offset above). Timestamped on
both sides (`t_send`, `t_recv`) so analysis can separate compute time from
network round trip. `++cycle` — the cycle counter increments after the
exchange, so `cycle` in the log means "exchanges completed".

### Lines 267–277 — fill this cycle's log row
Timestamps, then `t_prev = t_now` (**[edit-hazard, line 271]** — this
assignment must stay after `sample.dt_s` is computed from `t_prev` and
before the next cycle's dt measurement; moving it corrupts dt), then
`FillSample` (above), then the requested-vs-sent telemetry from
`actuation_status`.

### Lines 278–279 — the latched JOINT_FAULT note
Sticky bool: was the base's JOINT_FAULT summary bit ever set this run? Used
only for the post-run note (line 395). `kJointFaultBit` is the one base bit
`HasLiveFault` masks out as a stale historical aggregate (Safety.md).

### Lines 281–297 — edge-triggered fault-change printing
If either the base bank or any joint bank differs from last cycle, print the
decoded change (once per change, not per cycle) — capped at
`kMaxFaultChangePrints` (20) so a flapping bank cannot flood the terminal at
500 Hz; the CSV still has every cycle's banks. Visibility only — the
completed-sample priority decision comes next.

### Completed-sample freshness and stop priority
After `Send` returns, the Runner fills the row and prints any fault-bank
edge, then updates `FeedbackFreshnessMonitor` from that same reply. The
current `ack_unchanged_j*` counts are therefore present in the stop row.
`StaleAcknowledgementJoint` becomes active at 25 consecutive unchanged
replies (50 ms at 500 Hz); it is evidence that the downstream cyclic path
stalled, not evidence that a stationary joint failed to move.

The Runner computes every sample fact independently and passes them to
`ResolveStopPriority` (StopPriority.h). First true reason wins:

1. following error;
2. loss of LOW_LEVEL servoing;
3. a live fault, only when `kStopOnFault` enables fault stops;
4. the held-frame joint-warning boundary;
5. stale actuator acknowledgement.

An ignored live fault still sets the decision's sticky
`live_fault_observed` taint, but returns no stop reason; there is no
`ClassifyStop` call or ignored-fault `reason` reset in the Runner. Every
selected stop reason pushes this completed sample before leaving the loop.

### Decision-12 counter stops, then the normal push
Checked after the completed-sample priority above, so live state,
joint-boundary, and stale-feedback reasons keep priority.
`kNonFiniteStopCycles` (3) consecutive held cycles, or `kOverrunStopCycles`
(10) consecutive slow cycles, ends the run; `N <= 0` in Config disables one.
Each break pushes the sample first; otherwise the normal path pushes it once.

### Lines 342–348 — pacing
`sleep_until(next_cycle)` sleeps to the grid point; if the cycle overran the
grid (`next_cycle <= now`), the grid is re-anchored to `now` instead —
continue at rate rather than bursting to catch up. Bursting after a stall
would fire commands back-to-back at the arm.

### Lines 351–384 — the catch ladder
C++ tries `catch` clauses in order and takes the first matching type, so
order is specificity: `KDetailedException` (Kortex, decoded with its
sub-code) → `std::runtime_error` → `std::exception` → `...` (anything, even
non-exception types). Every clause does the same three things: set `reason`,
mark the reused `sample` as `refresh_ok = false` and push it (the last
partial row is the evidence), print. **[hides-work, lines 361–367]** The
`std::runtime_error` clause labels the stop "communication" — that is true
for the Hardware layer (whose failures throw runtime_error), but *any*
runtime_error from any module lands here and gets called a communication
failure. The catch-alls (371–384) exist so no exception type can skip the
teardown report — and, as the comment notes, the servoing restore no longer
depends on being caught at all, because the `ServoingGuard` destructor runs
during unwinding regardless.

### Lines 386–398 — teardown and the return
- **386–390** — after the catch ladder, `servoing_guard.Restore(std::cout)`
  explicitly restores SINGLE_LEVEL. It is outside the try, so every normal
  and caught-exception path reaches it; the `ServoingGuard` destructor is the
  retry backstop if that call fails. There is no actuation restore stage.
  **[edit-hazard, lines 386–390]** moving this call inside the try would skip
  the explicit restore on exception paths.
- **391–396** — the decoded stop report (`PrintStopReport`, Safety.md), the
  overrun tally, and the stale-JOINT_FAULT note if the summary bit was ever
  seen.
- **398** — the `LoopResult` aggregate: reason, taint flag, and the last
  sample's time/cycle (so they line up with the CSV's final row). Main turns
  this into the exit code: 0 only for `kUserStop` with no observed faults.
