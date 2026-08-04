# Runner.cpp + Runner.h — line-by-line read

**Entry point:** `Main.cpp:575` calls `RunControlLoop(...)` exactly once, after
the readiness gate, the CSV setup and the reference-source construction. This
is the function that MOVES THE ARM. Everything in this file exists to run the
takeover sequence T1–T5 (documented in the Runner.h banner, lines 6–24), the
500 Hz cycle, and the teardown D1–D3 on every possible exit path.

This doc follows real execution order: the header contract first, then
`RunControlLoop` top to bottom (which is also its execution order), with the
helper `FillSample` explained where the loop first calls it.

---

## Runner.h

### Lines 1–34 — the banner comment
Plain-English spec of the whole file: the takeover order (T1–T5), the per-cycle
order, and the teardown order (D1–D3). Not code, but treat it as normative —
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
  (State.h:114); at runtime this is either a `PoseTargetSource` (operator
  runs) or a `TrajectorySource` (playback runs). The Runner never knows
  which — that is the whole plug-in design.
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

### Lines 24–74 — `FillSample` (called once per cycle, line 286)
Copies one cycle's worth of data into a `LoopLogSample` (the 130-column CSV
row, Hardware.h:254). Field groups:

- **34–38** — desired and current end-effector position from
  `ControllerStatus` (what the controller computed this cycle).
- **39–42** — `sigma_min` (Jacobian conditioning), rotation error, and the
  measured tool quaternion. `status.tool_quat.coeffs()` is Eigen's internal
  x,y,z,w storage order — the comment is there because it is the opposite of
  the w-first constructor order and a classic source of silent bugs.
- **43–49** — builds the right-base origin from Config and sets
  `pd_beyond_reach`: is the desired position outside the arm's reach sphere
  minus a margin. <!-- FLAG hides-work: a "copy data into a row" helper is
  quietly computing a reach-policy judgement from Config constants; you would
  not look here to find where "beyond reach" is decided. --> **[hides-work,
  lines 43–49]** This is flag-only telemetry (targets are never rejected),
  but the computation lives in a function whose name promises pure copying.
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
- **70–73** — the trajectory-playback telemetry: the per-joint reference the
  command should land on, the playback clock and state. **[trajectory-only,
  lines 70–73]** For operator runs these stay NaN/0; they exist solely so a
  playback CSV can plot planned vs commanded vs measured.

### Lines 77–91 — entry and the T1 assertion
The signature repeats the header. Lines 89–91:

```cpp
assert(robot_ready);
if (!robot_ready)
    throw std::logic_error(...);
```

`assert` aborts the program in debug builds if the condition is false, and
compiles to *nothing* in release builds (when `NDEBUG` is defined) — which is
exactly why the `throw` follows it: the throw is the check that always runs.
**[unnecessary, line 89]** Given the unconditional throw on the next line,
the assert adds nothing except a louder crash in debug builds; it is
harmless, but the real guard is the throw (and, one level up, Main returning
before ever calling this function when the gate fails).

### Lines 93–109 — the compile-time stop-policy warnings
Three `std::cout` warnings, printed once before the loop, driven by
`config::kStopOnFault` and `config::kDisableFollowingErrorStop`. These are
`constexpr` values, so the compiler already knows which warnings print — the
point is that the *operator* knows too, at the terminal, before the arm
moves. The third (lines 106–109) fires only when both guards are off:
"YOU are the safety system." Nothing here changes behaviour; behaviour was
decided at compile time in Config.h.

### Lines 111–119 — loop-local state
- `nominal_dt_s` — 0.002 s, from the period.
- `reason = LoopStop::kUserStop` — the default outcome. Every stop path
  overwrites it; if nothing does, the loop ended because `stop` went true.
- `faults_observed` — sticky flag: any live fault ever seen taints the exit
  code even when the fault-ignoring policy keeps the loop running.
- `sample` — ONE `LoopLogSample`, reused every cycle. Reuse matters: the
  struct is big, and allocating per cycle inside a 2 ms loop is forbidden.
  **[edit-hazard, line 115]** Because it is reused, a field a cycle does not
  overwrite carries the *previous* cycle's value — the catch blocks (365–398)
  rely on this deliberately (they push the last partially-updated sample as
  evidence), but a new field added to `FillSample` that is only conditionally
  written would silently smear across cycles.
- `cycle`, `joint_fault_was_latched`, `counters` (the decision-12
  consecutive-cycle counters), `freshness_monitor` (ack-ID telemetry,
  Safety.md).

### Line 121 — `CyclicSession cyclic(base_cyclic);`
Constructs the object that owns the cyclic command frame (Hardware.h:146).
Construction is cheap and does not touch the network.

### Line 124 — T2: `ServoingGuard servoing_guard(base);`
**This line changes the robot's mode.** Constructing the guard sends
`SetServoingMode(LOW_LEVEL_SERVOING)` — from here until restore, this
process is the arm's controller and must stream commands. It is an RAII
object: *Resource Acquisition Is Initialization*, the C++ pattern where a
constructor acquires something and the destructor releases it, so the release
happens automatically on every way out of the scope, including exceptions.
**[edit-hazard, lines 121–124]** Order matters twice over: the guard must be
constructed *after* `cyclic` exists but *before* the `try` block (so the
destructor still restores single-level if anything below throws), and `Seed`
must come after it (commanding before the mode switch fails with
WRONG_SERVOING_MODE — that is the T2→T3 ordering in the banner).

### Lines 126–130 — the `try` and T3: `cyclic.Seed()`
Everything that can throw during the run is inside this one `try`. `Seed()`
is the program's single standalone feedback read inside the command loop.
**[hides-work, line 130]** `Seed` does more than read: it also initializes
the command frame's seven actuator slots (Hardware.h:140–144), and its
network round trip doubles as the settling time for the mode switch — the
comment says so, but the call looks like a plain read.

### Lines 132–137 — first `RobotState`
Copies the seed feedback into `state` (positions and velocities,
degrees→radians at this boundary). `RobotState` (State.h:30) is the small
Eigen struct every module downstream consumes.

### Lines 139–146 — fault-print seeding
`prev_joint_banks` / `prev_base_bank` record the fault banks *as they were at
entry*, so a bank already latched (which `RobotReadyForTakeover` tolerates
for the stale JOINT_FAULT summary) does not print as a fresh event on cycle
1. `fault_prints` caps the total number of fault-change prints (Safety.h:150).

### Lines 148–169 — T4: the holding frames and the seeding hand-off
This block is the heart of the takeover and the most order-sensitive code in
the file. **[edit-hazard, lines 148–169]**

- **148–151** — `commanded_deg` starts as the measured position (a "hold
  here" command). `commanded_velocity_deg_s{}` — the `{}` zero-initializes
  the array; without it a `std::array` of doubles holds garbage.
- **159** — first `Send(commanded_deg)`: a frame whose command equals the
  measurement. Its *reply* is a fresher measurement.
- **160–165** — that reply reseeds `state` AND `commanded_deg`, so the next
  three calls all see the exact same pose.
- **166–168** — the one-time seeding chain, in this order:
  `actuation.Prepare(state)` (integrator command = measurement — the ONLY
  time command state comes from measurement), `controller.Reset(state)`
  (captures the hold pose), `reference.Reset(state)` (source baseline; for
  playback this re-runs the start gate). Reordering these changes what each
  one captures. The Actuation.h and State.h contracts both name this order.
- **169** — one more holding frame; its reply becomes cycle 1's input, so
  the first control cycle starts from feedback that postdates all seeding.

The comment (154–158) records why this is two frames and not the old 0.5 s
handshake window — history you need before "simplifying" it further.

### Lines 171–178 — clocks and status
`std::chrono::steady_clock` is the monotonic clock (never jumps when the
wall clock changes — mandatory for control timing). `t_start` anchors all
CSV timestamps; `next_cycle` is the pacing grid; `status` is the
`ControllerStatus` telemetry struct, declared once outside the loop.

### Line 180 — `while (!stop)`
Reads the atomic every cycle. Ctrl+C (or SIGTERM) in Main sets it; the loop
notices within one cycle (≤ 2 ms) and exits with `reason` still at
`kUserStop` — the only successful outcome.

### Line 182 — `next_cycle += period;`
Advances the grid *first*, so pacing is "sleep until the next grid point",
not "sleep for a period after finishing" — errors do not accumulate.

### Lines 184–202 — dt measurement and overrun counting
- **187–193** — `dt_s` is the *measured* time since the previous cycle
  start, except cycle 0 uses the nominal 2 ms (there is no previous cycle),
  and it is clamped by `ClampedCycleDt` (Actuation.h:26) to at most 2×
  nominal — a scheduler stall must not integrate into one big position jump,
  which the base would fault on. dt is an input to the controller and the
  integrator, so it is sampled once, here, at cycle start.
- **194–202** — separately, the *unclamped* elapsed time is compared against
  `kOverrunFactor × nominal` (1.5× = 3 ms): a slow cycle increments both the
  consecutive counter (`counters.overrun`, reset to 0 by any healthy cycle)
  and the whole-run tally. Note the duplication: `t_now − t_prev` is computed
  twice (191 and 195) — same value, deliberate clarity over micro-saving.

### Lines 204–211 — refresh `state` from the previous exchange
The feedback consumed here was *received at the end of the previous cycle*
(or the T4 frame, on cycle 1). Degrees→radians again at the boundary, and
`state.t_s` becomes seconds since `t_start`. **[edit-hazard, lines 204–211]**
This one-cycle offset is a design fact the whole log annotates (Hardware.h
"cross-exchange row semantics"): in row i, the controller inputs come from
row i−1's reply. Moving the `Send` earlier or refreshing `state` later
changes the loop's phase and every offline analysis assumption.

### Lines 213–221 — reference, then controller
- **216** — `status = ControllerStatus{};` resets the telemetry struct to
  its defaults every cycle. **[edit-hazard, line 216]** The edge flags
  (`arrived_edge`, `playback_done_edge`, `playback_refused_edge`) are only
  ever *set*, never cleared, by the source/controller — this reset is what
  makes them edges. Remove it and every notice prints every cycle forever.
- **217–218** — `reference.Get(state, dt_s, status)`: the source says WHERE
  to be this cycle (pose channel, joint channel, or nothing = hold).
- **219–221** — `controller.DesiredVelocity(...)`: the law turns that into a
  desired joint velocity, rad/s, BEFORE any clamping (Controller.md).

### Lines 223–229 — capture the requested velocity for the record
Copies the controller's raw output into `requested_velocity_deg_s` *before*
the non-finite hold and the clamp can touch it — so the CSV's `reqvel_j*`
columns are the controller's true output, including a NaN if it produced one
(that NaN in the log is the evidence behind a `kNonFiniteCommand` stop).

### Lines 231–239 — the non-finite hold
`allFinite()` is Eigen's "no NaN, no infinity anywhere" check. A non-finite
velocity is replaced with zero (this cycle holds position) and the
consecutive counter increments; a finite cycle resets it. The bad value is
never integrated — the integrator would be poisoned permanently by one NaN.

### Lines 241–257 — edge-triggered prints
The arrival notice (243–247) and the two playback notices (248–257) print
*inside the cycle*, but only on the edge cycles, so the cost is bounded and
rare. The controller/source never print — they set flags, the Runner prints
(the "controllers do no I/O" rule). **[trajectory-only, lines 248–257]** The
refused/done notices exist solely for playback runs. **[edit-hazard, lines
241–257]** Any print added here that is *not* edge-triggered runs blocking
terminal I/O at 500 Hz and will cause overruns.

### Lines 259–271 — clamp, then integrate
- **263–268** — the per-joint speed clamp, the program's single speed limit:
  each velocity converted rad→deg, clamped to ±`qdot_limit_deg_s[i]` (45),
  converted back. **[edit-hazard, lines 263–268]** The double unit
  conversion is easy to break when editing — the limit array is in deg/s but
  the vector is rad/s; get one conversion wrong and the limit is off by a
  factor of 57. A clamp pinned at its limit is *allowed indefinitely*: far
  targets transit at clip speed by design (the comment records that the old
  saturation stop was removed).
- **269–271** — `actuation.Apply(...)` integrates
  `q_command += q̇·dt`, applies the command-lead bound (≤1° ahead of the
  measurement), and writes the result into `commanded_deg` /
  `commanded_velocity_deg_s` **through the reference parameters** —
  **[hides-work, lines 269–271]** the call *mutates* `commanded_deg`
  in place; nothing at the call site says "output parameter" except the
  Actuation.h signature. The returned `ApplyStatus` carries the pre-limiter
  setpoint and the per-joint "limiter engaged" flags for the log.

### Lines 273–279 — the one exchange
`cyclic.Send(commanded_deg)` writes the setpoints into the command frame,
stamps frame/command IDs, sends over UDP, and returns this cycle's feedback —
which the *next* iteration consumes (the phase offset above). Timestamped on
both sides (`t_send`, `t_recv`) so analysis can separate compute time from
network round trip. `++cycle` — the cycle counter increments after the
exchange, so `cycle` in the log means "exchanges completed".

### Lines 281–291 — fill this cycle's log row
Timestamps, then `t_prev = t_now` (**[edit-hazard, line 285]** — this
assignment must stay after `sample.dt_s` is computed from `t_prev` and
before the next cycle's dt measurement; moving it corrupts dt), then
`FillSample` (above), then the requested-vs-sent telemetry from
`actuation_status`.

### Lines 292–293 — the latched JOINT_FAULT note
Sticky bool: was the base's JOINT_FAULT summary bit ever set this run? Used
only for the post-run note (line 409). `kJointFaultBit` is the one base bit
`ClassifyStop` masks out as a stale historical aggregate (Safety.md).

### Lines 295–311 — edge-triggered fault-change printing
If either the base bank or any joint bank differs from last cycle, print the
decoded change (once per change, not per cycle) — capped at
`kMaxFaultChangePrints` (20) so a flapping bank cannot flood the terminal at
500 Hz; the CSV still has every cycle's banks. Visibility only — stopping is
`ClassifyStop`'s decision, next.

### Lines 313–329 — the stop decision
`ClassifyStop` (Safety.md) checks, in priority order: following error,
actuator fault, base fault (minus the JOINT_FAULT summary), left low-level
servoing. If it fires:

- a fault reason sets `faults_observed = true` (taints the exit code even if
  ignored);
- the loop breaks *unless* the reason is a fault AND `kStopOnFault` is false
  (the fault-ignoring experiment) — the sample is pushed before breaking so
  the stop cycle is always in the CSV;
- **line 328** — an ignored fault resets `reason` back to `kUserStop`.
  **[edit-hazard, lines 319–329]** This reset is load-bearing: without it, a
  transient ignored fault would leave `reason = kRobotFault` sitting in the
  variable, and a later Ctrl+C would be misreported as a fault stop. The
  taint survives in `faults_observed`; `reason` must describe why the loop
  actually *ended*.

### Lines 330–335 — freshness telemetry
`freshness_monitor.Update(...)` counts consecutive cycles each joint's
command-acknowledgement ID repeated; the counts go into the sample
(`ack_unchanged_j*`). Recorded, never acted on — the old stale-feedback stop
was removed 2026-08-03. **[edit-hazard, lines 319–335]** Note the ordering
wrinkle: when the loop breaks at line 325/326, that final sample was pushed
*before* this update ran, so the stop row's `ack_unchanged_j*` columns carry
the previous cycle's counts. Harmless today (telemetry), but anyone
tightening this into a stop again must move the update above the push.

### Lines 336–354 — the decision-12 counter stops, then the normal push
Checked AFTER `ClassifyStop` so the guard and live faults keep priority.
`kNonFiniteStopCycles` (3) consecutive held cycles, or `kOverrunStopCycles`
(10) consecutive slow cycles, ends the run; `N <= 0` in Config disables one.
Each break pushes the sample first. **[edit-hazard, lines 319–354]**
`log.push(sample)` appears four times (325, 344, 351, 354) — one per exit
path plus the normal path. A future early-`break` added between them that
forgets its own push loses the most important row of the run (the stop row).

### Lines 356–362 — pacing
`sleep_until(next_cycle)` sleeps to the grid point; if the cycle overran the
grid (`next_cycle <= now`), the grid is re-anchored to `now` instead —
continue at rate rather than bursting to catch up. Bursting after a stall
would fire commands back-to-back at the arm.

### Lines 365–398 — the catch ladder
C++ tries `catch` clauses in order and takes the first matching type, so
order is specificity: `KDetailedException` (Kortex, decoded with its
sub-code) → `std::runtime_error` → `std::exception` → `...` (anything, even
non-exception types). Every clause does the same three things: set `reason`,
mark the reused `sample` as `refresh_ok = false` and push it (the last
partial row is the evidence), print. **[hides-work, lines 375–381]** The
`std::runtime_error` clause labels the stop "communication" — that is true
for the Hardware layer (whose failures throw runtime_error), but *any*
runtime_error from any module lands here and gets called a communication
failure. The catch-alls (385–398) exist so no exception type can skip the
teardown report — and, as the comment notes, the servoing restore no longer
depends on being caught at all, because the `ServoingGuard` destructor runs
during unwinding regardless.

### Lines 400–413 — teardown D1→D2→D3 and the return
- **401** — D1: `actuation.Restore()` (currently a no-op — POSITION mode
  holds the last setpoint — but the hook point is contractual).
- **405** — D2: `servoing_guard.Restore(std::cout)` — the *explicit* restore
  to SINGLE_LEVEL, done here rather than left to the destructor so any
  Kortex error is printed to stdout with its sub-code and the base gets its
  documented settling wait; the destructor remains the retry backstop.
  **[edit-hazard, lines 400–405]** D1 before D2 is the contract (Actuation.h
  says Restore runs BEFORE the servoing guard restores); and this teardown
  is *outside* the try, so it runs on every path — moving it inside the try
  would skip it on the exception paths.
- **406–411** — D3: the decoded stop report (`PrintStopReport`, Safety.md),
  the overrun tally, and the stale-JOINT_FAULT note if the summary bit was
  ever seen.
- **413** — the `LoopResult` aggregate: reason, taint flag, and the last
  sample's time/cycle (so they line up with the CSV's final row). Main turns
  this into the exit code: 0 only for `kUserStop` with no observed faults.
