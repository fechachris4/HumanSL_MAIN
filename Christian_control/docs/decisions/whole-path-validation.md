# Whole-path validation before further Kinova motion

Date: 2026-08-04

Status: completed; historical evidence for the retained controller limits and
target circuit. The recorded starting pose is not a runtime prerequisite.

## Problem statement

Incrementally changing the compiled fixed target made the supervised recovery
runs non-comparable. The logs also show three materially different takeover
orientations. An instantaneous Jacobian probe or a collection of individually
reached waypoints does not establish that one fixed-orientation target circuit
is feasible.

No further controller motion is permitted until an offline rollout has used
the production kinematics, controller law, saturation, position integration,
command-lead limiter, and bounded-joint guard over every cycle of the complete
path.

The recorded initial pose and joint branch below document one validated
experiment. The reusable controller must not require the arm to be at that
pose: normal startup measures the current pose, holds it during takeover, and
profiles from it to the configured terminal target.

## Global constraints

- Right arm only, `192.168.1.10`; Cartesian quantities are expressed in the
  right-arm `base_link` frame and positions are in metres.
- Kortex actuator order is J1 through J7. Controller joint calculations use
  radians; configured and reported joint bounds and velocities use degrees.
- The latest measured initial state is taken from
  `runs/2026-08-04/loop_log_20260804_162345.csv` at `t=47.4105 s`:

  ```text
  p = [0.0036998, -0.7105000, 0.2508000] m
  q = [75.3150, 57.8941, 189.5530, 289.9420, 266.4060, 41.0442, 332.3960] deg
  qdot = [0, 0.000607678, -0.000303151, 0, 0, 0, -0.000606301] deg/s
  quaternion xyzw = [-0.279020, -0.787391, 0.508398, 0.209032]
  ```

- The orientation above was the first registered candidate. It was held
  immutable during H1--H4, then rejected at Fixed. A new registered
  orientation was selected from measured feasible data before later trials.
- Controller gains and switches remain `Kp_position=10 1/s`,
  `Kp_rotation=10 1/s`, `Kd_position=0.5`, `Kd_rotation=0.5`,
  `lambda=0.1`, orientation enabled, velocity feedback enabled, and null-space
  centering disabled.
- The controller runs at 500 Hz. Joint-rate saturation remains 45 deg/s,
  command lead remains 1 deg, and the software position boundaries remain
  J2 +/-126.9 deg, J4 +/-145.0 deg, and J6 +/-118.0 deg.
- Offline terminal acceptance requires position error no greater than 1 mm
  and fixed-orientation error no greater than 0.001 rad. These are evaluated
  together before the two-second hold begins.
- Automatic stops for faults, following error, stale acknowledgements,
  non-finite output, repeated overruns, lost low-level mode, communication
  failure, and outward bounded-joint crossings must be active for validation.
- The immutable target sequence is:

  ```text
  Fixed  [ 0.3834, -0.4051, 0.7225]
  T1     [-0.3834, -0.2051, 0.7225]
  T2     [ 0.3000, -0.4000, 0.5500]
  T3     [ 0.4300, -0.4000, 0.5500]
  Return [ 0.3834, -0.4051, 0.7225]
  ```

- No intermediate waypoint may be selected from hardware trial and error.
- No hardware executable may run during the offline tasks below.
- Collision clearance is not an offline acceptance claim. The repository has
  collision meshes and Coal, but no validated SRDF disabled-pair matrix and no
  environment model.

## Falsifiable hypotheses

### H1: current step-reference feasibility

The exact current 6-DoF DLS pipeline can complete the recovery leg from the
latest state and the entire target-and-return circuit while holding the
approved orientation, without crossing a bounded-joint software boundary.

Reject H1 if an ideal one-cycle kinematic follower fails to settle within
1 mm and the configured orientation tolerance, hits a joint guard, produces a
non-finite value, or makes negligible progress while the Jacobian loses rank.
Velocity saturation by itself does not reject geometric feasibility; it is a
separate command-shaping result.

### H2: shaped-reference suitability

If H1 establishes a feasible joint branch, a straight Cartesian path with a
seventh-order rest-to-rest scalar profile reduces or removes joint-rate
saturation and respects the same pose and joint constraints without changing
the path or controller gains.

The first fixed candidate limits are 0.05 m/s Cartesian speed, 0.10 m/s^2
Cartesian acceleration, and 0.50 m/s^3 Cartesian jerk. The profile is
`s(u)=35u^4-84u^5+70u^6-20u^7`; segment duration is the smallest duration that
satisfies all three analytical bounds. Each final target must hold for at
least two seconds.

Reject H2 if any shaped leg fails H1's terminal criteria, activates a bounded
joint guard, or requires persistent rate saturation. A changed limit is a new
explicit offline experiment, not an in-place adjustment.

### H3: command-lead robustness

A 12-cycle (24 ms at 500 Hz) feedback-delay sensitivity rollout, selected as
the conservative end of the 10--24 ms clean-run response envelope, completes
the same path without the 1 deg command-lead
limiter causing a terminal failure or the 3 deg following-error boundary being
approached.

This is sensitivity evidence only. It is not a dynamics, torque, payload, or
collision guarantee.

## Task 1: exact offline cycle seam

1. Add a pure, fixed-size joint-rate clamp used by both `Runner.cpp` and the
   offline rollout so the production limit cannot drift from the analysis.
2. Write the failing hardware-free test first, then make the smallest
   mechanical production substitution.
3. Rebuild and run all registered hardware-free tests.

## Task 2: Cartesian profile and arrival semantics

1. Add pure seventh-order time-scaling logic with analytical duration bounds,
   zero endpoint velocity/acceleration/jerk, and finite-input rejection.
2. Test one-dimensional and 3-D endpoints, derivative bounds, zero-length
   motion, invalid inputs, profile completion, terminal-only arrival, and the
   two-second hold before advancement.
3. Keep all terminal parsing, I/O, allocation, and locking outside the 500 Hz
   loop. The source may use only preallocated state and fixed-size Eigen data.

## Task 3: exact whole-path rollout and report

1. Build a hardware-free C++ executable using the production
   `TrackingController`, `DualArmKinematics`, `PositionIntegration`, rate clamp,
   and configured 2 ms integration.
2. Run both the current step references and the shaped references from the
   latest measured state with the approved explicit orientation.
3. Primary plant assumption: a one-cycle ideal position follower. Secondary
   sensitivity: a documented conservative feedback delay. Keep the two
   results separate.
4. Emit a CSV and summary containing reference and actual Cartesian paths,
   joint trajectories, J2/J4/J6 signed margins, all singular values and raw
   condition number, requested/clipped/applied velocities, saturation and
   command-lead activity, software-guard events, position/orientation
   residuals, and per-target terminal status.
5. State that the mixed translational/angular Jacobian scaling makes its raw
   singular values and condition number scale-dependent.

## Task 4: classify the offline evidence

1. Test H1 alone. If rejected, identify whether the first failure is target
   pose infeasibility, a path singularity, a bounded-joint branch, or a
   controller defect.
2. Only after H1 classification, test H2. Do not change the geometric path.
3. Only after a shaped ideal rollout passes, run H3 sensitivity.
4. If the fixed orientation is infeasible, stop controller implementation and
   perform an offline orientation/pose search as a new documented hypothesis.

## Task 5: validation configuration

Only after the offline path passes:

1. Restore the original compiled Fixed target and set `kStopOnFault=true`.
2. Make the approved quaternion explicit for every circuit reference.
3. Add startup gates for Cartesian position, orientation, and the predicted
   redundant joint branch. The existing 0.15 m position gate must not be
   widened to force the recovery leg.
4. Feed the immutable target file through the existing mailbox or provide an
   equally deterministic preallocated source; do not introduce a second
   controller implementation.
5. Rebuild, run the full test suite and static analysis, review the complete
   diff, and prepare the experiment sheet before any connection.

## Results and retained decision

- H1 step control and H2's initial 0.05/0.10/0.50 profile both failed at
  Fixed with about 22 mm position residual, about 3.45 mrad orientation
  residual, J6 about 5 deg from its software boundary, and a nearly singular
  Jacobian. Removing the step discontinuity removed saturation but did not
  remove the failure.
- A deterministic 774-trial bounded multi-start search found no Fixed
  solution for the first orientation, while T1/T2/T3 had hundreds. Fixed was
  therefore the orientation-specific incompatibility.
- The previously measured Fixed quaternion
  `[0.320642, 0.413930, -0.703345, 0.480788]` xyzw had bounded solutions at
  all four unique endpoints. The complete straight-path circuit passed.
- The retained slower profile is 0.025 m/s, 0.05 m/s^2, and 0.25 m/s^3.
  Its 24 ms sensitivity rollout completed with zero velocity saturation and
  lead limiting. The post-hardware stable-return branch improved the offline
  minimum raw sigma to 0.1081, maximum mixed-unit condition to 16.92, and
  minimum bounded-joint margin to 13.52 deg.
- The validation log is
  `runs/2026-08-04/validation_circuit_final.csv`: two consecutive complete
  circuits, nine post-start arrival events, 185,716 control cycles, zero
  faults, refresh failures, acknowledgement stalls, lead-limit events,
  overruns, or inferred dropped cycles. Maximum following error was 0.206
  deg. Measured bounded-joint margins were J2 57.63 deg, J4 11.85 deg, and
  J6 12.84 deg. Minimum raw sigma was 0.02259. The real closed-loop response
  lag estimate was 144 ms; raw/lag-compensated RMS Cartesian errors were
  2.13/0.85 mm, while every reported terminal crossing was inside 1 mm and
  1 mrad.
- Final startup is registered to the stable returned Fixed branch and its
  measured quaternion. The controller refuses takeover outside 1 mm,
  1 mrad, or 1 deg wrapped joint error.
- Temporary rollout, multi-start IK, read-only preflight, and high-level
  reposition executables were removed after they produced the evidence.
  The production target source, profile, startup gate, process lock, and
  hardware-free regression tests remain.

## Hardware acceptance

Every hardware run requires Christian present, a clear workspace, and the
e-stop immediately available. Its experiment sheet must state the hypothesis,
exact command, initial joint branch, fixed quaternion, path/profile, predicted
joint extrema and margins, success metrics, abort criteria, and telemetry
fields. Prediction and measurement must be compared quantitatively before a
second run. Final success requires the same immutable configuration to repeat
the full circuit with target errors, joint margins, timing, faults,
acknowledgements, and dropped-sample evidence reported.
