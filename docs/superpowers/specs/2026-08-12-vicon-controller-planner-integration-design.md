# Vicon integration into the controller and planner

Date: 2026-08-12
Status: design approved by Christian 2026-08-12 (interactive review); no
implementation authorized yet. Each stage below is its own approval gate.

## Goal

In Christian's words: plan the integration of the Vicon segments, markers,
the origin, and anything else those require, into the current controller and
planner.

Unpacked: bring live Vicon data — the five Nexus segment poses, the marker
data as diagnostics, and the Vicon world origin related to the robot's
`mount` frame — into both consumers on the restored planner–controller
boundary, so that:

- the controller can hold or track an end-effector pose in the world frame
  while the wearer and backpack move (the chicken-head role), and
- the planner can accept world-frame targets and convert them correctly at
  plan time.

This is the first slice of the 2026-08-10 target architecture re-proposed
after the 2026-08-12 rollback (commit d04b2035), exactly as that rollback
intended: "re-propose slices on the new boundary, Vicon/chicken-head first."

## Why

The wearer moves during and after any trajectory, so a live world-frame
estimate of the mount is the missing input on both sides. Base compensation
is control, not planning; frame conversion at plan time is planning, not
control. Keeping those on their own sides of the boundary is what the
rollback restored and what this design preserves.

## Decisions made at design review (Christian, 2026-08-12)

- **Scope: both consumers, staged.** One design covering controller and
  planner, delivered as small stages, each with its own sign-off.
- **Transport: provider seam + replay** (option C). The controller is
  written against a base-motion provider interface with live, replay, and
  static implementations. Rejected: a separate sensing process feeding the
  controller (adds a hop of latency and a serialization format on the
  control path — it survives as panel diagnostics only), and a seamless
  in-process link (untestable without the lab).
- **Planner scope: plan-time frames only.** World-frame targets are
  converted with the `world_T_mount` estimate current at plan request.
  Asynchronous replanning while the base moves is explicitly a later slice.
- **Replay infrastructure: yes, minimal.** One recorder and one replay
  source, just enough that stages 2–4 develop offline after one lab capture.

## Current state this builds on (verified 2026-08-12)

- `Christian_control/vicon/` is the standalone sensing wrapper:
  `ViconInterface` with marker access and (uncommitted) `getSegmentPoses()`
  plus its CTest; `connect_vicon` prints markers and segments. No Kortex on
  its link line, by construction.
- The controller's reference contract after the rollback is
  `ReferenceSource::Get` returning optional `PoseReference` / optional
  `JointReference` (`basic_control/src/State.h:158`). The Cartesian PD law
  and its tests exist but are unwired because nothing produces
  `PoseReference` yet.
- Live check today: hosts `.206` and `.210` answer ping but port 801 is
  closed on both — the DataStream server is not currently streaming, so
  "all five segments now track" is Christian's report, not yet verified.
  On 2026-08-11 the server delivered 20 labelled markers with all five
  segments occluded.
- The 2026-08-11 read-only Vicon monitor design (panel `VICON` tab +
  sensing-only helper) exists as a spec but is not implemented. It is
  complementary diagnostics, not part of this control path.

## The frame contract

Notation: `A_T_B` is the pose of frame B in frame A (`Eigen::Isometry3d`,
metres, radians). Millimetres exist only at the SDK read and are converted
exactly once.

Three kinds of frames, none coinciding by default:

- **`vicon_world`** — the Vicon global origin from the lab wand
  calibration. The axis convention is whatever Nexus is configured for;
  `SetAxisMapping` has never been called in any project code, so stage 0
  sets or asserts it explicitly.
- **Nexus segment frames** — the five segments of subject
  `Dr Octopus Christian`: `Mount`, `LeftBase`, `RightBase`, `LeftEE`,
  `RightEE`, each streaming as `world_T_segment`. A segment's origin and
  axes come from its Nexus template, chosen by whoever built the subject —
  they are not URDF frames and must never be treated as such.
- **URDF frames** — `mount` (root link, midpoint of the two base origins),
  `base_link` / `leftbase_link`, and the tool/flange links. The
  mount-to-base transforms are known exactly from the URDF (and are
  themselves inherited rather than surveyed — a discrepancy found later is
  a finding, not a failure).

The single constant this integration hangs on is the calibration transform
**`mountseg_T_mount`** — the URDF `mount` frame expressed in the Nexus
`Mount` segment frame. Once measured, the live estimate both consumers use
is:

```
world_T_mount(t) = world_T_mountseg(t) · mountseg_T_mount
```

Measurement method (stage 2): the URDF gives `mount_T_leftbase` and
`mount_T_rightbase` exactly; the `LeftBase` and `RightBase` segments give
the same structures in `vicon_world`. A static recording over N frames
yields `mountseg_T_mount` by least squares, with the fit residual reported
always — a graded quality measure, never a pass/fail. The same applies to
each base segment's own template offset: `LeftBase`/`RightBase` segment
frames relate to URDF base links through the same calibration, not by
assumption.

Independent cross-check: `LeftEE`/`RightEE` give Vicon's world EE pose to
compare against `world_T_mount · mount_T_ee(q)` computed by Pinocchio FK
from logged joint angles. This validates the whole chain with evidence the
calibration itself did not create. (Right arm ends at `ConfiguredTool_Link`,
left at the bare flange — the two "EE" points are different points on
different arms and are never compared to each other.)

Physical fact to confirm in the lab, not from code: the `Mount` segment
markers must be on the rigid backpack plate. If they are on the wearer's
body the transform varies with posture and the whole scheme is invalid;
the 2026-08-08 audit's stationary-arms/moving-wearer recording experiment
settles this from a recording alone.

## Stages

Each stage is separately approved before implementation. No stage runs a
robot-facing binary; stage 3's hardware bring-up is its own explicitly
authorized session.

### Stage 0 — live verification and frame contract (lab, sensing-only)

Restore streaming (Nexus running; port 801 reachable — Windows firewall
was the culprit on 2026-08-10). Verify with `connect_vicon`: all five
segments valid and moving plausibly. Set or assert the axis convention.
Write down each segment template's origin/axis choice as built in Nexus.
Capture two recordings: one fully static, one with arms stationary while
the wearer moves (the Mount-rigidity experiment). Deliverable: a short
decision record plus the recordings. Code changes: none beyond what exists.

Acceptance: five valid segments observed live; conventions recorded;
both recordings captured and readable.

### Stage 1 — snapshot, recorder, replay (minimal, in `Christian_control/vicon/`)

A `ViconSnapshot` value: frame number, host receive time, frame rate,
latency, segment poses with validity, markers as diagnostics; metres;
quaternions accepted only when finite and near-unit-norm, never silently
normalised. A recorder writing the house run-log CSV convention and a
replay source yielding the same snapshot type. Blocking acquisition lives
on its own thread wherever it is used; nothing here touches the controller.

Acceptance: hardware-free CTest coverage — record→replay round-trip
faithful on every field; occlusion, staleness, truncated-file and
unknown-format cases degrade with a stated reason. A stage-0 recording
replays offline.

### Stage 2 — calibration tool (offline, against the stage-0 recording)

A tool computing `mountseg_T_mount` (and the base-segment offsets) from a
static recording plus the URDF, writing a calibration file with residuals
and provenance (which recording, how many frames, what residual). Runs the
EE cross-check against a logged run. Eigen-only — the GTSAM/Pinocchio
translation-unit conflict forbids mixing; Pinocchio enters only in this
tool.

Acceptance: synthetic exact-recovery test (a perfect observation built
from the URDF yields zero residual — proving the arithmetic measures the
rig, not itself); an injected perturbation is recovered exactly; the real
recording yields a calibration whose EE cross-check error is reported with
stated uncertainty.

### Stage 3 — controller slice: the chicken-head hold (Level 2)

Re-propose the base-motion seam on the restored boundary, this time
approved up front:

- `BaseMotionSource` in `basic_control`: estimate = `world_T_mount` pose,
  twist, timestamp, valid. Providers: static identity (regression:
  bit-identical behaviour with Vicon absent), replay, and live Vicon — an
  acquisition thread in the controller process handing the latest validated
  estimate to the 500 Hz loop through a non-blocking slot. No blocking
  Vicon I/O on the control thread, ever.
- A reference source that anchors the end-effector's current world pose at
  activation (`world_T_ee = world_T_mount · FK(q)`) and re-expresses it in
  the mount frame each cycle, producing the `PoseReference` that wires up
  the existing Cartesian PD law through the existing limits, integration
  and safety path. Both controller laws keep sharing that single path.
- Degradation: a stale or invalid estimate freezes compensation at the
  last good value and logs a graded freshness/quality measure. It never
  causes an abrupt stop; sustained poor quality is escalated by report,
  not by veto.

This stage gets its own design gate before implementation, offline tests
against replay, the safety-review checklist, one adversarial review
(motion-path code), and a separately authorized supervised hardware
bring-up with Christian present.

### Stage 4 — planner: plan-time frames

`planner_bridge` accepts a world-frame target plus the `world_T_mount`
estimate current at plan request, converts once to the mount frame the
planner already works in, and records in the plan metadata which estimate
was used. GPMP2 stays outside the control loop, unchanged. Replanning
triggered by base motion is out of scope — a later slice with its own
design.

Acceptance: bridge tests with a known transform show a world target and
its hand-converted mount equivalent produce the same plan; a plan's
metadata names the estimate it used.

## Data flow

```
Vicon DataStream (100 Hz, mm, vicon_world)
  → acquisition thread: validate, convert to m, timestamp      [stage 1/3]
  → BaseMotionEstimate slot (latest value, non-blocking read)  [stage 3]
      → controller: world-anchored PoseReference each cycle
      → freshness/quality logged; stale ⇒ freeze, never stop
  → recorder / replay (same snapshot type)                     [stage 1]
  → calibration tool (offline)                                 [stage 2]
  → planner_bridge: world target → mount target at plan time   [stage 4]
```

## Constraints carried by this design

- No target in `Christian_control/vicon/` links Kortex, verifiable from
  the link line alone.
- New Vicon/calibration code is Eigen-only so it can be linked from either
  side of the GTSAM/Pinocchio divide.
- GPMP2 remains outside the 500 Hz loop; blocking I/O remains off the
  control thread; both controller laws pass through the same velocity
  limits, joint handling, integration and safety path.
- Units, frames and timestamps are explicit at every boundary; mm→m
  happens exactly once.
- No robot-facing binary is executed without Christian's explicit
  authorization for that run; building is never running.

## Open questions (tracked, not blocking the spec)

1. Are the `Mount` segment markers on the rigid plate? Stage 0's
   moving-wearer recording answers this; everything downstream assumes yes
   until it does.
2. What axis convention is the lab's Nexus configured for? Stage 0 asserts
   it; code assumes nothing until then.
3. Twist in the estimate: finite-differenced segment poses with light
   smoothing, or omitted (valid-flagged) in the first slice? Decided at
   stage 3's design gate.
4. Which host is canonical — `.206` (code default) or `.210` (observed
   2026-08-10)? Stage 0 records the answer; the default follows it.

## What this design deliberately does not do

No replanning supervisor, no feasibility margins, no Kalman filtering, no
force plates, no changes to `ViconInterface`'s existing marker behaviour,
and no revival of the rolled-back posture-guidance or pose-primary modes.
Each of those is future work with its own approval.
