# Why the reactive controller stops on far targets

Learning record, 2026-08-13. Diagnosed offline from the 2026-08-05 run
logs; no hardware used. Status: root cause confirmed; fix chosen but not
yet implemented (nearest-achievable projection — see the end).

## Reported symptom

Commanding the Cartesian reactive-pose controller ("chicken-head" law) to
a target far from the current pose ends the run early — experienced as
"the arm faults".

## Expected behaviour

The arm moves toward the target and reaches it, or gets as close as it
can.

## What actually happens — confirmed from logs

It is not a robot fault. Five runs on 2026-08-05 ended with
`exit_reason = joint_limit_warning`: the controller's own guard held the
last safe command and stopped the loop when a bounded joint reached its
outward software boundary (`kJointSoftwareLimitDeg`). The robot's fault
bank stayed clean. In `loop_log_20260805_010730.csv`, joint 6 sat pinned
at exactly −118.0° (its boundary) from the first cycles until the stop at
t = 2.27 s.

## Confirmed root cause

**The target position was reachable; the orientation the controller was
holding made it infeasible.** The reactive law solves a 6-DoF task:
position plus the tool orientation captured at takeover, held fixed. For
this start pose that orientation was rpy (7.5°, −13.0°, 171.2°) in the
right base frame.

Evidence, from `probe_path_reachability` (offline IK, never connects to
the robot), target (0.083, −0.02, 0.7225) m in right_base, 0.297 m from
the start pose:

| Orientation held | Result |
|---|---|
| takeover (7.5, −13.0, 171.2)° | pose solves to 0.2 mm — but only with **j4 ≈ 11.5° past its limit**. Infeasible. |
| (0, 0, 180)° | same failure, j4 by ~10° |
| (0, −45, 180)° | **reachable, all joints inside limits** |
| (30, −30, 150)° | reachable |
| (7.5, −60, 171.2)° | reachable |

Pitching the tool down by roughly 45° buys the whole position back. The
position was never the problem.

## Why the live run pinned j6 when IK says j4

The IK result is the true constrained solution: it needs j4 out of range.
The running controller never sees that solution — it follows the local
gradient (damped least squares) downhill from the start configuration,
and on that path joint 6, the joint with the narrowest range (±118°
usable), hit its boundary first. Different joint, same underlying fact:
with the orientation pinned, every route to this target crosses a bounded
joint's limit. In the 010730 run, j6 *started* at −117.99° — an earlier
attempt that night had already parked it on the boundary.

## Hypotheses tested and rejected

- *The target is outside the position workspace* — rejected: three
  different orientations reach it with all joints inside limits.
- *A firmware fault (velocity, following error) ends the run* — rejected:
  exit_reason in every log is the software joint-limit-warning stop;
  fault banks clean. (Real firmware joint-limit faults did occur on
  2026-08-03, but those were the degenerate 0/0 threshold bands, fixed
  the same day.)

## Independent corroboration

The same failure mode was measured on 2026-08-06 for a left-arm goal:
position reachable, requested orientation impossible — joint 6 needed
~38° past its stop (recorded in `probe_path_reachability.cpp`'s header).
And the known 2026-08-05 stall (null-space centering gain 23 freezing the
arm 218 mm short) is the other face of the same conflict: stop or stall,
the task demands what the joint limits refuse.

## The chosen fix (approved 2026-08-13, not yet implemented)

Project an infeasible target to the nearest achievable pose, go there,
and report the shortfall — never a silent substitution. The diagnostic
sharpens what "nearest" must mean: nearest in *pose* space, not position
space. A position-only projection would have concluded "unreachable,
stop short" — wrong, since the position is fully reachable if the
controller yields ~45° of tool pitch. So the projection must be able to
trade orientation for position, with the trade-off reported.

## Report-ready summary

A supernumerary-limb controller holding the takeover orientation as a
hard 6-DoF constraint refuses targets its arm could reach: for a 0.30 m
reach, offline inverse kinematics showed the commanded pose solvable to
0.2 mm but requiring joint 4 eleven degrees past its limit, while
relaxing tool pitch by 45° made the same position reachable within all
joint limits. The live controller, descending this infeasible gradient,
drove its narrowest-range joint (6) to the software boundary, where the
joint-limit guard correctly stopped the run. The failure is therefore a
task-specification problem — orientation over-constraint — not a
workspace, gain, or safety-threshold problem, and the remedy is graded
constraint relaxation (orientation demoted from hard constraint to
ranked objective) rather than wider limits.

## Remaining limitations

- The probe samples a handful of orientations; the full feasible
  orientation set at a given position was not mapped. A sweep over pitch
  at fixed position would make a good thesis figure.
- The 2026-08-05 runs predate the current reference-source architecture;
  the reactive mode is at present unwired in production, so the confirmed
  behaviour is of the law, not of today's binary end-to-end.
