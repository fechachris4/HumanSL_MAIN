# Config.h — line-by-line

File: `basic_control/src/Config.h`. A header has no execution order — every
value here is fixed at compile time and simply *read* by the rest of the
program. So this read goes top to bottom, but for each value it says **who
reads it and when** during a real run, which is the execution-order story
that matters.

One C++ idea covers almost every line: `inline constexpr`. `constexpr`
means the value is computed at compile time and can never change at
runtime. `inline` (on a variable, since C++17) means the header can be
included by many .cpp files and they all share one definition instead of
colliding at link time. Net effect: these are true compile-time constants —
changing any of them means **recompiling**, and the running binary can be
older than the value you think it has (the freshness gate below exists
because exactly that bit once).

Flag categories used inline: **trajectory-only**, **unnecessary**,
**hides-work**, **mixed-jobs**, **edit-hazard**.

---

### Lines 1–7 — banner
The key sentence: "--log is the only runtime argument, so these values are
the only thing between the operator and the arm: read this file before a
session." That is the file's design contract — configuration is code, so
every change is a commit, reviewable and attributable. Rationale documents
live in `../../docs/decisions/`.

### Line 9 — `#pragma once`
Include guard: the header's contents are pasted in at most once per .cpp
file, however many times it is `#include`d.

### Lines 11–13 — includes
`<array>` for JointVector, `<chrono>` for the typed durations,
`<cstddef>` for `std::size_t`.

### Line 22 — `using JointVector = std::array<double, 7>;`
The joint-space value type used everywhere. `std::array<double, 7>` is a
fixed-size array that knows its size: "exactly 7 values" is enforced by the
compiler, and index 0 = joint 1 ... index 6 = joint 7, matching Kortex's
`feedback.actuators(i)` order. A type alias, not a new type — any
`std::array<double, 7>` is a JointVector. This is the one thing in the file
that is not a setting; it lives here so every module shares it without
extra headers.

### Line 28 — `namespace config`
Everything below is `config::kSomething` at the point of use, so a reader
of Main.cpp always knows a value came from this file.

### Lines 30–35 — reference source selection
- **Line 34** — `kReferenceSource = "operator"`: which reference source
  main() builds — `"operator"` (typed/fixed pose targets) or
  `"trajectory"` (pre-validated joint trajectory). Read once at
  Main.cpp:250–251; a typo throws before any hardware contact. Note it is a
  C string compared with `std::string` at runtime — the *validation* of
  this compile-time value happens at startup, not compile time.
- **Line 35** — `kTrajectoryFile`. **FLAG trajectory-only:** only read when
  the source is `"trajectory"` (load at Main.cpp:263, echoed in the CSV
  preamble). The path is relative to the working directory the binary runs
  from — an edit-worthy detail, but the loader's error message covers it.

### Lines 37–41 — right-arm identity
`kRightRobotIp` is the only address the program ever connects to;
`kRightBaseFrame` / `kRightEndEffectorFrame` name URDF frames and are
validated when `DualArmKinematics` is constructed (Main.cpp:300) — a typo
fails before connection. Every printed pose and every typed target is in
this base frame.

### Line 45 — `kLeftNominalRad`
The left arm exists only inside the 14-DoF model; these seven radians are
what the model assumes its joints are frozen at. No left connection,
feedback, or command anywhere in the program. Changing this changes where
the model *believes* the left arm is — which today affects nothing the
controller outputs (only right-arm frames are used), but would matter the
day anything queries left-arm or inter-arm geometry.

### Lines 47–52 — session credentials and timeouts
`admin`/`admin` is the Kinova factory default; the timeouts are how long
the base keeps an idle session (60 s) or connection (2 s) alive before
dropping it. Read once by `Connect` (Hardware.h). The short connection
timeout is what makes a crashed program's session evaporate quickly instead
of blocking the next run.

### Lines 54–60 — `kModelVelocityLimitsDegS`
**FLAG edit-hazard (lines 57–60):** the program's *single* speed limit —
the clip applied to every commanded joint velocity. Currently a TEMPORARY
uniform 45 deg/s, deliberately below the model's rated 79.64/69.91 limits;
the comment points at `qdot-limit-raise.md` and says raising it back is a
deliberate decision, not a cleanup. Because `kQdotLimitDegS` below is
defined *equal to this*, editing these numbers changes the actual hardware
clip. There is no Cartesian speed limit anywhere (see line 112's comment),
so this array is the whole speed story.

### Lines 62–68 — timing
`kControlDtS = 0.002` is the single source of truth; the frequency (500 Hz)
and the `std::chrono::microseconds` cycle period both *derive* from it, so
the three can never disagree. `std::chrono::microseconds` is a typed
duration — passing it around instead of a bare number means the compiler
catches unit mistakes. Read by the runner (loop grid), the banner print,
and the log-buffer sizing (where Main.cpp:413's integer division is the
edit-hazard — see Main.md).

### Lines 70–77 — `kUseFixedTarget`
The big behavioural switch for operator runs. `false` (current): the arm
holds at takeover and moves only when a target is typed — the safe default,
because a typed target is chosen while looking at the freshly printed
current pose. `true`: no stdin thread; THE ARM DRIVES TO THE COMPILED
TARGET IMMEDIATELY after takeover. The comment carries the safety
reasoning: a typed target cannot go stale the way a compiled one can.

### Line 84 — `kMaxFixedTargetDistanceM = 0.15`
The freshness gate's limit (Main.cpp:513): refuse the run if the compiled
fixed target is more than 15 cm from the *measured* end-effector at
startup. Exists because of a real incident (2026-08-04, 37 cm of drift
between compile and run) — the control law drives the whole gap at clip
speed from cycle one. Raising this number widens how wrong a stale target
can be before anyone is warned.

### Lines 86–106 — the fixed target itself
- **Line 102** — `kFixedTargetM = {0.3834, -0.4051, 0.7225}`.
  **FLAG edit-hazard (lines 95–102):** the value is inseparable from the
  long comment above it — this target is −3 cm in z from a *specific
  measured pose on a specific date*, chosen because `probe_direction` showed
  −z drives joint 6 INWARD, the only safe direction while j6's limit band
  cannot be restored (see lines 151–156). Joint 6 has only ~12° of inward
  window. Editing the numbers without redoing that analysis (re-probe after
  the arm moves, keep moves small) is exactly the mistake the comment is
  guarding against; the 0.15 m gate catches *stale* targets, not *unsafe
  directions*.
- **Lines 103–106** — `kFixedTargetUseRpy = false` keeps the takeover
  orientation; `true` ALSO rotates the tool to `kFixedTargetRpyRad`
  (R = Rz·Ry·Rx, matching the startup printout's convention so values can
  be pasted straight across). The default rpy is the Home orientation, so
  at Home it commands no rotation — a safe default for the dangerous
  option.

### Line 112 — `kKpCartesian = 10.0`
The position-error gain: commanded velocity = 10 × position error (1/s).
At 10 cm of error that is 1 m/s — the comment is honest that this is
aggressive and that the per-joint clip is what actually bounds speed. Read
by the reactive law every cycle. Also the reminder that no Cartesian
velocity/acceleration/workspace limiting exists at all.

### Line 116 — `kDlsLambda = 0.1`
Damped-least-squares damping for the Jacobian inverse. Plain English:
near a singular pose (arm stretched out, wrists aligned) the pure inverse
demands huge joint speeds; λ trades tracking accuracy for bounded, well-
conditioned motion. Larger = slower but safer near singularities. Also
damps the null-space projector, so it is doing two jobs with one number.

### Lines 118–126 — reactive-law gains and toggles
`kKpRotation` (orientation error gain), `kKdPosition`/`kKdRotation`
(velocity-error damping), `kNullGain` (centering strength), and the three
enable flags. Current staging: orientation and the Kd term ON, null-space
centering OFF. These map one-to-one onto terms in ReactiveLaw.h; the
toggles exist so each term was brought up on hardware separately.

### Lines 128–134 — null-space centering targets
Midpoints (all 0) and the mask selecting bounded joints 2/4/6 only
(continuous joints can't be "centered"). The CAUTION comment is the real
content: this arm's *configured soft limits* sit far inside the URDF range,
so centering joint 4 toward 0 can push it into its limit — confirm in the
web dashboard before ever flipping `kNullSpaceEnabled` to true. Currently
dormant (the toggle is false).

### Lines 136–140 — reach telemetry
Base origin, 0.902 m max reach, 5 cm margin. FLAG ONLY in the code's own
words: targets beyond reach are flagged in telemetry, never rejected or
projected. Nothing here stops the arm.

### Line 144 — `kQdotLimitDegS = kModelVelocityLimitsDegS`
**FLAG edit-hazard:** two names for one value — the "model limits" and the
"clip applied before integration" are defined equal *by construction* so
they cannot disagree. The hazard is the day someone makes them differ:
every consumer must then be re-audited for which of the two it should read
(the runner takes `kQdotLimitDegS`, Main.cpp:578; the trajectory gate
derives from it via line 242). Until then it reads as duplication —
resist "simplifying" it away *and* resist splitting it casually.

### Lines 146–158 — JOINT_LIMIT thresholds
The firmware backstop bands, re-applied on EVERY connection because SDK
writes do not survive a power cycle.
**FLAG edit-hazard + hides-work (lines 157–158):** the arrays encode three
things in one place: (a) magnitude in degrees, sign applied per HIGH/LOW by
`EnsureJointLimits`; (b) **0 means "leave that joint alone"** — a sentinel,
not a limit of zero degrees (an actual 0/0 band is the degenerate state
that faults all outward motion!); (c) the current values are working around
live hardware state — only joint 4 (145/150) is restored, and joint 6 is
deliberately 0 since 2026-08-04 because its config service is wedged and
every RPC to it times out (~15–20 s of dead wait per run, buying nothing).
Consequence spelled out in the comment: j6's band stays presumed-degenerate,
ONLY INWARD j6 MOVES ARE SAFE, run `probe_direction` before choosing
targets, restore 118/123 when j6 answers again. Someone filling in "the
missing limits" for joints 1–3/5/7, or restoring j6 without checking the
service, changes startup behaviour on real hardware.

### Line 164 — `kStopOnFault = false`
**FLAG edit-hazard:** in caps in the source for a reason: A LIVE FAULT
DOES NOT END THE RUN. Faults are still decoded, printed, logged and taint
the exit code (Main.cpp:619–626), but the loop keeps commanding.
ATTENDED USE ONLY — the operator is the stop. Deliberately compile-time
only, never runtime-settable, so changing it is a visible commit.

### Lines 166–202 — the guard overrides
All three default false = every guard active; each is echoed into the CSV
preamble so a run recorded with one enabled is identifiable forever after.
- **Line 184 — `kAllowUnverifiedActuators = false`.** true = startup gates
  accept an actuator whose config service does not answer, instead of
  refusing takeover (the unreachable joint then runs with mode unverified
  and limit band unknown). History in the comment: turned on for wedged
  joint 6 on 2026-08-04, then back off the same night once j6 was excluded
  from the gate via its zero entry at line 157 — the override "has nothing
  to excuse" now. Reads as dormant, but it is the *first* knob to reach for
  if another joint's config service wedges.
- **Line 192 — `kSkipStartupGates = false`.** true = skip both gates for
  every joint; critically the j4/j6 bands are then NEVER re-applied and
  revert to 0/0 on power cycle → firmware faults outward motion. The
  comment steers you to the narrower override above instead. The loud
  warning it triggers is at Main.cpp:339–343.
- **Line 202 — `kDisableFollowingErrorStop = false`.** true = the loop
  never stops on following error. The comment does the safety arithmetic
  for you: combined with `kStopOnFault = false` and the no-motion stops
  removed 2026-08-03, the ONLY remaining automatic stop would be loss of
  low-level servoing — operator plus firmware limits become the entire
  safety margin. The limit value itself (line 214) keeps feeding telemetry
  either way.

### Lines 204–209 — consecutive-cycle stop counters
`kNonFiniteStopCycles = 3`: three consecutive cycles of non-finite (NaN/inf)
controller output stop the run — and a non-finite cycle is never
integrated, the setpoint just holds. `kOverrunStopCycles = 10` /
`kOverrunFactor = 1.5`: ten consecutive cycles measuring more than 3 ms
(1.5 × 2 ms) stop the run — a persistently late loop is a controller whose
timing assumptions are false. N ≤ 0 disables a counter.

### Line 214 — `kFollowingErrorLimitDeg = 3.0`
Stop when any joint's |commanded − measured| exceeds 3°, chosen to fire
before the base's own ~5° ejection (which would end servoing abruptly).
Passed into `RunControlLoop` at Main.cpp:579. But read the next flag —

### Lines 216–223 — command shaping
**FLAG edit-hazard (lines 222–223):** `kCommandLeadLimitDeg = 1.0` caps how
far the integrated setpoint may lead measured position (constructor arg of
`PositionIntegration`, Main.cpp:543); `kNullRampDurationS` fades centering
in over 1 s. The trap is the *interaction* the comment documents: because
1° < the 3° following-error guard, the lead limiter always engages first
and **the following-error guard can never fire while the limiter is
active** — nothing stops a frozen joint by itself. Tuning either number
independently silently changes which mechanism is in charge: raise the lead
limit above 3° and the following-error stop suddenly becomes reachable —
new behaviour nobody asked for. These two constants and line 214 are one
coupled system wearing three names.

### Line 228 — `kArrivalToleranceM = 0.001`
Purely informational: one printed line the first time the EE comes within
1 mm of a new target. The comment pre-authorises raising it toward 5 mm if
the notice comes late or never. No control effect.

### Lines 230–236 — playback gains
**FLAG trajectory-only (lines 235–236):** `kPlaybackKp = 0.5` corrects the
wrapped reference-minus-measured error during playback (1/s);
`kStartMismatchLimitDeg = 0.2` is the per-joint gate that the arm already
*is* at the trajectory's first row — checked before takeover
(Main.cpp:353–376) and again at `TrajectorySource::Reset`, where failing is
a permanent hold. Unused on operator runs.

### Lines 238–247 — trajectory validation gates
**FLAG trajectory-only (lines 242–247):** `kTrajectoryVelGateFactor = 0.9`
(a file may use at most 90 % of the command clip, so the runner's clamp can
never engage on a validated file — the gates and the clip are deliberately
coupled through line 144), `kTrajectoryAccelLimitDegS2` (Kinova Table 43),
and `kTrajectoryPosLimitDeg` (Table 39 wrapped ranges for bounded joints
2/4/6; **0 here means "continuous joint, no position check"** — the same
0-as-sentinel convention as the limit bands, worth knowing before editing).

### Lines 249–255 — logging
`kLogBufferSeconds = 30` sizes only the handoff queue between loop and
writer thread — slack for a disk hiccup, not a retention limit (the CSV on
disk is unbounded; ~175 KB/s at 500 Hz, prune `runs/` yourself).
`kLogDrainInterval` = writer wakes every 100 ms; `kLoopLogPrefix` names the
default CSV files.

### Line 256 — end of namespace config.

---

## Flag summary for this file

| Lines | Flag | Reason |
|---|---|---|
| 35 | trajectory-only | trajectory file path, read only in playback |
| 57–60 | edit-hazard | the program's single speed limit; 45 is a deliberate temporary derate |
| 95–102 | edit-hazard | fixed target valid only for one probed pose/date; j6 inward-only window |
| 144 | edit-hazard | alias equality is a design invariant, not duplication |
| 157–158 | edit-hazard, hides-work | 0 = "skip joint" sentinel; current values encode the wedged-j6 workaround |
| 164 | edit-hazard | faults do not stop the run; attended use only |
| 222–223 | edit-hazard | 1° lead limit makes the 3° following-error stop unreachable |
| 235–236 | trajectory-only | playback gains |
| 242–247 | trajectory-only | trajectory validation gates (0 = continuous-joint sentinel) |
