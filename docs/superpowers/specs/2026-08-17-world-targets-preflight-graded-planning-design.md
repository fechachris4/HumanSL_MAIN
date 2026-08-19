# World Targets, Pre-Flight Visibility, and Graded Planning

**Date:** 2026-08-17
**Status:** Draft — awaiting Christian's approval. No code changes yet.
**Prompted by:** the three rejected circle sessions of 2026-08-17 and
Christian's own framing afterwards: "it is too easy for the planner to
reject a trajectory I've requested, and it's too easy for me not to know
where the points I've requested are in space", plus "the target should be
in world pose so now we can get a real error"
(raw-prompt-log 2026-08-17 20:44:40, 23:36:16; interactive answers the
same evening: world-fixed targets confirmed, all three capabilities
designed together).

## 1. The evidence this design answers

All from runs/2026-08-17 and the current source, session of 2026-08-17:

- At 14:10 a left-arm circle plan passed validation (e_command 4.6 mm,
  joint-limit margin +24.9 deg, start error 0.000 deg). After the rig was
  re-hung (mount world orientation changed 174 deg between Vicon
  snapshots) and the arm re-parked, every session failed — including two
  with the identical goal file. Three supervised hardware sessions were
  spent discovering this.
- The circle is authored as bare mount-frame numbers in goal.yaml.
  Nothing showed that the enlarged circle's far side lies 0.905 m from
  the left base (near full stretch) with a fixed orientation, or that its
  centre sits 1.18 m from the tool's parked position. Both facts were
  knowable offline.
- The planner solved something every time (GPMP2 converged; time scaling
  found an executable pace), then a binary verdict discarded all of it
  and emitted nothing. This contradicts the repository's own
  graceful-degradation principle and the approved nearest-achievable-pose
  goal (story.md, approved 2026-08-13 ~16:10).

Goals, stated as outcomes:

1. A target can be fixed in the room, so the error the system reports is
   the true world-stabilisation error while the wearer moves.
2. A goal edit shows, before any hardware session, where the requested
   points are relative to the arm and whether the plan would be accepted.
3. The planner answers an infeasible request with the best achievable
   limit-respecting trajectory and an honest shortfall report, never a
   bare refusal. Which shortfalls may execute stays Christian's call.

## 2. Part A — world-frame targets

**Change.** goal.yaml blocks (point goals, circle paths, and future path
types) accept `frame: world` alongside the existing mount / right_base /
left_base. A world-framed target is used as-is; no mount conversion at
plan time.

**Frames and symbols.** World = Vicon world W. Mount = M, with T_W_M the
Vicon-measured mount pose (position m, quaternion) already snapshotted
per planning request. Tool = E. The controller's reported error for a
world target is

    e_p = p_d^W − p_WE(t)        (m, world frame)
    e_R = log(R_d^W · R_WE(t)ᵀ)  (rad, world frame)

which is already the production law's error once the reference is world —
no control-law change is required. What changes is only where the
reference numbers come from: authored directly in W instead of being
produced from mount numbers × T_W_M at plan time.

**Why this yields a "real" error.** A mount-declared target moves with
the wearer between sessions and between replans: re-hanging the rig on
2026-08-17 silently relocated the goal. A world-declared target is the
same point in the room every run; wearer motion then produces genuine
tracking error rather than a moved goal.

**Staleness.** Planning against a world target requires a T_W_M snapshot
only to compute the start-relative approach and reachability — the
target itself needs no Vicon. The existing plan-time snapshot and its
sequence/age reporting are unchanged. The controller's existing
freshness gates (world_fresh_max_age_s, prolonged-stale handling) remain
the runtime authority; this design adds nothing to the 500 Hz path.

**Panel.** The goal editor displays every target in both frames — the
authored one and its conversion via the latest logged T_W_M — read from
what the controller logged, computing nothing of its own (the
one-live-chain rule, story.md 2026-08-13).

## 3. Part B — pre-flight check on goal edit

**Change.** Saving a goal in the panel (or pressing an explicit Check
button) runs the existing offline bridge (`planner_preview`: full solve
+ validation, output to stdout, no controller handoff, no Kortex, no
motion) and shows:

- the full validation report, identical in format to the session report;
- a "where is this" summary computed by the same planner model FK, not a
  parallel implementation: per-sample distance from the arm base as a
  fraction of maximum tool reach (min/max over the path), distance and
  orientation change from the current tool pose, and the target in every
  relevant frame.

**Inputs.** Measured start q from the most recent run CSV (the same
FindLatestRunCsv mechanism the bridge already uses); latest T_W_M from
the controller's log. If either is missing, the check degrades to a
stated nominal start and says so in the output — it never silently
pretends to a measured state (graceful degradation, honest labels).

**Guarantee.** The pre-flight path can never command hardware: it is the
existing preview entry point, which emits text only. This must be stated
in the panel UI next to the result.

## 4. Part C — spatial view

**Change.** A goal-preview plot in the panel's PLOTS tab, following the
existing pattern (a Python script generates the figure; the panel only
displays it): the requested path samples, the arm base frames, the mount
frame, the current tool pose from the latest log, and the reach envelope
(sphere at maximum tool reach, with a marked comfortable band), drawn in
the frame the goal is authored in plus a world view. Geometry comes from
the preview run's machine-readable output; the plotting script computes
no kinematics of its own.

## 5. Part D — graded planning instead of refusal

**Verdict split.** The report's checks divide into two classes:

*Safety-critical — hard, unchanged, can never be overridden:*
non-finite values; modelled-geometry collision; joint position limits in
the emitted trajectory; start-state splice guard; velocity/acceleration
above the firmware-faulting limits. A trajectory failing any of these is
never emitted (today's −51 deg joint-limit trajectory stays dead).

*Quality — graded, reported, no longer veto:* task fidelity against the
requested pace and shape (e_command), time-scaling stretch beyond the
requested duration, closure drift. For these the planner emits the best
safety-respecting trajectory it found, plus a shortfall report: achieved
vs requested lap time, max/rms/p95 position shortfall, orientation
shortfall, and where along the path the worst point lies.

**Repair mechanisms, in order of application:**

1. *Pace repair (exists):* uniform time scaling — same path, slower.
   Its result is now a graded outcome ("lap takes 102 s instead of the
   requested 12 s"), not a fidelity failure.
2. *Shape repair (new):* when the requested path is not reachable as a
   pose path, project each sample to the nearest achievable pose,
   yielding orientation before position, per the approved
   nearest-achievable-pose goal — and report the shortfall explicitly.
3. *Constrained re-solve* is out of scope for this slice and recorded as
   a future option.

**Who decides execution.** Emission is not execution. A quality
shortfall above configurable thresholds (planner.yaml, e.g.
max_accepted_stretch, max_accepted_shortfall_m/rad) requires explicit
confirmation in the panel/session flow before the controller activates
the trajectory; below the thresholds it activates as today. Headless
(non-interactive) runs treat above-threshold shortfalls as a stop with
the report printed — conservative default, because nobody is present to
confirm.

## 6. Invariants preserved

- The pose/twist-only planner boundary, the in-process handoff design,
  and the controller's law, limits, and safety path are untouched.
- GPMP2 stays outside the 500 Hz loop; the pre-flight and spatial view
  run entirely offline.
- The panel computes nothing of its own: preview binary and controller
  logs remain the only sources.
- No check that can fault the arm is weakened; the change moves *quality*
  judgments from veto to report, never safety ones.

## 7. Hardware-free verification plan

- Unit tests: world-framed goal parsing (point + circle), rejection of
  ambiguous blocks, frame conversion equivalence (mount-authored target
  converted at plan time == the same target authored in world given the
  same T_W_M).
- Preview integration test: a goal known-infeasible at pace (today's
  15:59 configuration, replayed from the run CSV) yields an emitted
  time-scaled trajectory + shortfall report, with
  hardware_execution_allowed reflecting safety checks only and the
  confirm-threshold flag set.
- Shape-repair test: today's radius-0.2 circle yields a projected path
  with reported shortfall, never a 180 deg orientation error in the
  emitted command.
- Pre-flight determinism: two runs of the check on the same inputs give
  identical reports (the fixed-seed rule already in place).
- Panel tests extend the existing suite for the new editor fields, check
  button, and plot registration.

## 8. Out of scope

- Constrained re-solve inside GPMP2; replanning-rate changes.
- The joint-6 world-hold stop of 2026-08-17 (correct guard behaviour;
  re-anchoring/derating semantics remain separate design work).
- Any hardware run; physical validation of world-authored targets waits
  for a separately authorized supervised session.

## 9. Open decisions (each with a recommendation)

1. World-goal authoring aid: pick coordinates by clicking in the spatial
   view vs typing numbers. Recommend: typing first, view read-only; a
   click-to-set control is a later panel slice.
2. Threshold defaults for confirm-vs-auto: recommend max stretch 1.25×,
   max position shortfall 10 mm, max orientation shortfall 5 deg, all in
   planner.yaml where the panel already edits.
3. Whether the old mount-frame circle stays the committed example in
   goal.yaml or the file switches to a world-framed example after the
   first successful world run. Recommend: keep mount example until a
   world target has been validated in pre-flight against real logs.
