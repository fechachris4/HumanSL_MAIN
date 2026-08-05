# Config.h — line-by-line

File: `basic_control/src/Config.h`, at HEAD after commit f64325c0 ("remove
trajectory playback: fixed-target-only controller"). A header has no
execution order — every value here is fixed at compile time and simply
*read* by the rest of the program. So this read goes top to bottom, but for
each value it says **who reads it and when** during a real run, which is
the execution-order story that matters.

One C++ idea covers almost every line: `inline constexpr`. `constexpr`
means the value is computed at compile time and can never change at
runtime. `inline` (on a variable, since C++17) means the header can be
included by many .cpp files and they all share one definition instead of
colliding at link time. Net effect: these are true compile-time constants —
changing any of them means **recompiling**, and the running binary can be
older than the value you think it has, which is why every run records its
effective configuration in the CSV.

Since the playback removal there is no reference-source choice and no
`kUseFixedTarget` switch: every run profiles from its measured startup pose to
the compiled terminal target.

Flag categories used inline: **unnecessary**, **hides-work**,
**mixed-jobs**, **edit-hazard**.

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

### Lines 30–34 — right-arm identity
`kRightRobotIp` is the only address the program ever connects to;
`kRightBaseFrame` / `kRightEndEffectorFrame` name URDF frames and are
validated when `DualArmKinematics` is constructed (Main.cpp:229–232) — a
typo fails before connection. Every printed pose and the compiled target
are in this base frame.

### Line 38 — `kLeftNominalRad`
The left arm exists only inside the 14-DoF model; these seven radians are
what the model assumes its joints are frozen at. No left connection,
feedback, or command anywhere in the program. Changing this changes where
the model *believes* the left arm is — which today affects nothing the
controller outputs (only right-arm frames are used), but would matter the
day anything queries left-arm or inter-arm geometry.

### Lines 40–45 — session credentials and timeouts
`admin`/`admin` is the Kinova factory default; the timeouts are how long
the base keeps an idle session (60 s) or connection (2 s) alive before
dropping it. Read once by `Connect` (Hardware.h). The short connection
timeout is what makes a crashed program's session evaporate quickly instead
of blocking the next run.

### Lines 47–53 — `kModelVelocityLimitsDegS`
**FLAG edit-hazard (lines 50–53):** the program's *single* speed limit —
the clip applied to every commanded joint velocity. Currently a TEMPORARY
uniform 45 deg/s, deliberately below the model's rated 79.64/69.91 limits;
the comment points at `qdot-limit-raise.md` and says raising it back is a
deliberate decision, not a cleanup. Because `kQdotLimitDegS` below is
defined *equal to this*, editing these numbers changes the actual hardware
clip. There is no Cartesian speed limit anywhere (see line 96's comment),
so this array is the whole speed story.

### Lines 55–61 — timing
`kControlDtS = 0.002` is the single source of truth; the frequency (500 Hz)
and the `std::chrono::microseconds` cycle period both *derive* from it, so
the three can never disagree. `std::chrono::microseconds` is a typed
duration — passing it around instead of a bare number means the compiler
catches unit mistakes. Read by the runner (loop grid), the banner print,
and the log-buffer sizing (where Main.cpp:283–284's integer division is the
edit-hazard — see Main.md).

### The terminal target
- `kFixedTargetM` is the terminal position in the right-arm `base_link` frame,
  in metres. The startup pose is measured at runtime, so this value is no
  longer a startup freshness gate or a registered IK-branch requirement.
- The target remains safety-relevant: the low-level controller follows a
  bounded Cartesian profile, while the independent joint-rate, following-error,
  reach telemetry, and joint-boundary guards remain active.
- **Lines 87–90** — `kFixedTargetUseRpy = false` keeps the takeover
  orientation; `true` ALSO rotates the tool to `kFixedTargetRpyRad`
  (R = Rz·Ry·Rx, matching the startup printout's convention so values can
  be pasted straight across). The default rpy is the Home orientation, so
  at Home it commands no rotation — a safe default for the dangerous
  option.

### Line 96 — `kKpCartesian = 10.0`
The position-error gain: commanded velocity = 10 × position error (1/s).
At 10 cm of error that is 1 m/s — the comment is honest that this is
aggressive and that the per-joint clip is what actually bounds speed. Read
by the reactive law every cycle. Also the reminder that no Cartesian
velocity/acceleration/workspace limiting exists at all.

### Line 100 — `kDlsLambda = 0.1`
Damped-least-squares damping for the Jacobian inverse. Plain English:
near a singular pose (arm stretched out, wrists aligned) the pure inverse
demands huge joint speeds; λ trades tracking accuracy for bounded, well-
conditioned motion. Larger = slower but safer near singularities. Also
damps the null-space projector, so it is doing two jobs with one number.

### Lines 121–128 — reactive-law gains and toggles
`kKpRotation` (orientation error gain), `kKdPosition`/`kKdRotation`
(velocity-error damping), and the three enable flags —
`kOrientationEnabled`, `kVelocityTermEnabled`, `kNullSpaceEnabled`. Current
staging: orientation, the Kd term, and null-space limit avoidance are all
ON. These map one-to-one onto terms in ReactiveLaw.h; the toggles exist so
each term was brought up on hardware separately.

### Lines 184–192 — deadband null-space limit avoidance
`kLimitAvoidZoneDeg` (20°) and `kLimitAvoidGain` (2.0, 1/s) replaced the
midpoint-centering targets (`kNullGain`/`kNullMidpointDeg`/
`kNullCenteringMask`) on 2026-08-05 — centering pulled every bounded joint
everywhere, and the damped projector leaked enough of it into task space to
stall the arm 218 mm short of a target; see
`docs/superpowers/specs/2026-08-05-null-space-limit-avoidance-design.md`.
The replacement objective is exactly zero except inside the deadband
`[kJointSoftwareLimitDeg − kLimitAvoidZoneDeg, kJointSoftwareLimitDeg]` per
bounded joint (2/4/6 only; the continuous joints' `kJointSoftwareLimitDeg`
entries are 0, the sentinel `LimitAvoidanceVelocity` reads as "skip"), where
it pushes inward at `kLimitAvoidGain × excess` — at the limit itself that
is gain × zone ≈ 40°/s before projection and the per-joint clip. Anchoring
the zone to `kJointSoftwareLimitDeg` (Lines 163–182) instead of an
independent constant means the avoidance zone and the software stop it
backs up cannot drift apart.

### Target validation
The stdin parser validates target syntax and finite coordinates only. There is
no reach sphere or reach-screen telemetry in the controller. Reachability,
collision clearance, joint-limit clearance, and path safety remain separate
operator or planning responsibilities.

### Line 128 — `kQdotLimitDegS = kModelVelocityLimitsDegS`
**FLAG edit-hazard:** two names for one value — the "model limits" and the
"clip applied before integration" are defined equal *by construction* so
they cannot disagree. The hazard is the day someone makes them differ:
every consumer must then be re-audited for which of the two it should read
(the runner takes `kQdotLimitDegS`, Main.cpp:403). Until then it reads as
duplication — resist "simplifying" it away *and* resist splitting it
casually.

### Lines 130–142 — JOINT_LIMIT thresholds
The firmware-enforced joint-position bands, re-applied on EVERY connection
because SDK writes do not survive a power cycle. There is no separate
client-side joint-position clamp: the software warning guard instead holds
the full last-safe frame and stops before an outward warning crossing, while
allowing inward recovery. The velocity clip, reach screen, and following-
error stop are distinct protections.
**FLAG edit-hazard + hides-work (lines 141–142):** the arrays encode three
things in one place: (a) magnitude in degrees, sign applied per HIGH/LOW by
`EnsureJointLimits`; (b) **0 means "leave that joint alone"** — a sentinel,
not a limit of zero degrees (an actual 0/0 band is the degenerate state
that faults all outward motion!); (c) non-zero values deliberately cover
every bounded joint: joint 2 (130/140), joint 4 (145/150), and joint 6
(118/123), all warning/error magnitudes in degrees. Warnings are j2's
established 130 deg and j4/j6 145/118 deg (inside their documented
147.8/120.3 deg ranges); errors 140/150/123 deg are outside the documented
128.9/147.8/120.3 deg ranges. A live read found j2's thresholds back at
0/0, so it is now re-applied and verified on every connection alongside
j4/j6. Someone filling in "the missing limits" for continuous joints
1/3/5/7 changes startup behaviour on real hardware.

### `kStopOnFault = true`
**FLAG edit-hazard:** a live base or actuator fault ends validation motion.
Faults are decoded, printed, logged, and taint the exit code. This policy is
deliberately compile-time only, never runtime-settable, so changing it is a
visible source edit and rebuild.

### Lines 150–186 — the guard overrides
All three default false = every guard active; each is echoed into the CSV
preamble so a run recorded with one enabled is identifiable forever after.
- **Line 168 — `kAllowUnverifiedActuators = false`.** true = startup gates
  accept an actuator whose config service does not answer, instead of
  refusing takeover (the unreachable joint then runs with mode unverified
  and limit band unknown). Keep it false: the current contract requires
  j2/j4/j6 warning and error thresholds to read back before takeover. It is
  only an explicit last-resort override for a failed configuration service.
- **Line 176 — `kSkipStartupGates = false`.** true = skip both gates for
  every joint; critically the j2/j4/j6 bands are then NEVER re-applied and
  revert to 0/0 on power cycle → firmware faults outward motion. The
  comment steers you to the narrower override above instead. The loud
  warning it triggers is at Main.cpp:268–272.
- **`kDisableFollowingErrorStop = false`.** true = the loop never stops on
  following error. The retained validation configuration leaves both this
  guard and `kStopOnFault` active. Loss of low-level
  servoing, the software joint-boundary guard, and enabled non-finite and
  overrun counters still do. The limit value itself (line 192) keeps feeding
  telemetry either way.

### Lines 184–187 — consecutive-cycle stop counters
`kNonFiniteStopCycles = 3`: three consecutive cycles of non-finite (NaN/inf)
controller output stop the run — and a non-finite cycle is never
integrated, the setpoint just holds. `kOverrunStopCycles = 10` /
`kOverrunFactor = 1.5`: ten consecutive cycles measuring more than 3 ms
(1.5 × 2 ms) stop the run — a persistently late loop is a controller whose
timing assumptions are false. N ≤ 0 disables a counter.

### Line 192 — `kFollowingErrorLimitDeg = 3.0`
Stop when any joint's |commanded − measured| exceeds 3°, chosen to fire
before the base's own ~5° ejection (which would end servoing abruptly).
Passed into `RunControlLoop` at Main.cpp:420. But read the next flag —

### Lines 194–201 — command shaping
**FLAG edit-hazard (lines 200–201):** `kCommandLeadLimitDeg = 1.0` is the
lead-projection target from wrapped measured position (constructor arg of
`PositionIntegration`, Main.cpp:397); `kNullRampDurationS` fades the
null-space limit-avoidance gain in over 1 s. Each already-clamped qdot first
forms a proposed command; when
its lead exceeds the threshold, the lead projection targets exactly 1° from
wrapped measurement. A final envelope limits the sent delta to
`abs(qdot_clamped * dt)`, so it wins when discontinuous feedback would
require a larger recovery jump. Actual lead can therefore temporarily exceed
1°, and the unchanged 3° following-error stop remains reachable as the
backstop. These constants and line 192 are one coupled system: tuning
either changes how quickly lead recovery can occur and when that backstop
can matter.

### Lines 203–206 — `kArrivalToleranceM = 0.001`
Purely informational: one printed line the first time the EE comes within
1 mm of a new target. The comment pre-authorises raising it toward 5 mm if
the notice comes late or never. No control effect.

### Lines 208–213 — logging
`kLogBufferSeconds = 30` sizes only the handoff queue between loop and
writer thread — slack for a disk hiccup, not a retention limit (the CSV on
disk is unbounded; ~175 KB/s at 500 Hz, prune `runs/` yourself).
`kLogDrainInterval` = writer wakes every 100 ms; `kLoopLogPrefix` names the
default CSV files.

### Line 220 — end of namespace config.

---

## Flag summary for this file

| Lines | Flag | Reason |
|---|---|---|
| 50–53 | edit-hazard | the program's single speed limit; 45 is a deliberate temporary derate |
| 79–86 | edit-hazard | terminal target and its direction remain hardware-affecting |
| 128 | edit-hazard | alias equality is a design invariant, not duplication |
| 141–142 | edit-hazard, hides-work | 0 = "skip joint" sentinel; current values encode the wedged-j6 workaround |
| 148 | edit-hazard | faults do not stop the run; attended use only |
| 200–201 | edit-hazard | 1° lead projection target; the final rate envelope can defer recovery above 1°, leaving the 3° following-error stop reachable as backstop |
