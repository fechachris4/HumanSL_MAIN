# Cartesian path tasks, phase 1: geometry, frame crossing, reachability probe

Learning record, 2026-08-07. Follows the protocol in the repository root
`CLAUDE.md`.

## Why

Every task the stack could express was a single goal pose, reached by whatever
route the optimiser preferred. Drawing a circle inverts that: the shape
*between* the endpoints is the deliverable. Phase 1 builds the geometry, the
frame crossing a path needs, and — before any planning — a way to ask whether
the arm can actually trace the shape.

The design constraint was Christian's: Vicon and MPC arrive later, and the
work must not have to be re-plumbed for each. The resolution is that new code
goes through the right seam while the legacy conversion sites are left alone;
a feature does not have to refactor its predecessors to avoid adding to them.

## What was built

- `CartesianPath` (`planner_bridge/src/CartesianPath.h`): an ordered, timed
  sequence of poses **carrying its own frame**. That last part is what stops
  frame conversions leaking into every call site as new frames appear. It
  mirrors `msc_project`'s `sample(t) -> FramedTarget`. Eigen-only, so it is
  includable from the gtsam and Pinocchio sides alike.
- `GenerateCircle`: centre, radius, plane normal, sample count, start angle,
  and an orientation policy — `kFixed` (hold one orientation, what drawing on
  a surface wants) or `kRadialInward` (+z aimed at the centre). Constant
  angular rate, so the trace is constant-speed. Refuses a spec that cannot
  describe a circle rather than silently degenerating.
- `PathFrames` (`PoseToMount`, `PathToMount`): the single crossing into
  `mount`. Converting a whole pose is ONE homogeneous multiply, which gets
  translation and rotation right together — `BridgeMain`'s existing
  `ToMount`/`RotationToMount` split them only because its inputs arrive
  separately. A room/world frame from Vicon composes here and nowhere else.
- `probe_path_reachability`: solves IK at every sample with **continuation
  seeding** (each solve seeded from the previous solution, so the arm stays on
  one IK branch instead of flipping its elbow mid-path) and reports which
  samples are on the path. Never connects to a robot.
- `test_cartesian_path`: recovers the circle's radius, plane, closure, and
  spacing from the emitted samples rather than re-deriving them with the
  generator's own arithmetic.

## Confirmed finding: the IK's tolerances measured the solver, not the arm

The probe's first run reported all 13 samples of a 3 cm circle "reachable"
while showing 8.6–19.4 mm of position error. Both statements came from the
solver's own acceptance test, and neither meant the circle would be traced.

**Evidence.** The residuals formed a hard ceiling at 20 mm with nothing above
it — the signature of a solver stopping, not of an arm failing to reach.
`POSITION_TOLERANCE` was 2e-2 m: the solve exits the moment it is within
20 mm. Orientation errors were all under 0.3°, so only the position half was
being abandoned.

**Why the loose value was ever right.** The solver's original job was to seed
a point-to-point optimiser that carries its own goal term and corrects the
endpoint afterwards, so stopping at 20 mm cost nothing. Nothing corrects the
middle of a traced path, so the same value certifies a 19 mm-wide smear as a
circle.

**Fix.** `IKTolerance` — convergence and acceptance thresholds as parameters,
defaulted to the four values the solver has always used, so every existing
caller is bit-for-bit unchanged. The probe asks for a tenth of the tolerance
it is certifying.

**Result.** The same 3 cm circle: worst residual 19.4 mm → **0.10 mm**, 13/13
on path. A 10 cm circle at the same centre: 0.19 mm, 13/13. The circle was
always traceable; the number was measuring the wrong thing.

## Verification

- `ctest`: planner_bridge 11/11, basic_control 11/11.
- Point-to-point regression: the known left-arm case still reports
  `solved IK pose` at 1.34181 mm, identical to before.
- Discrimination, so the probe is not merely passing everything: a circle
  requesting an orientation measured at a different point is refused outright
  (0/13, "part of this path is not reachable at all"), while good circles pass.
- Nothing has run on hardware.

## Remaining limitations

- Reachability is not executability. The probe says a configuration exists at
  each sample; it says nothing about collisions, joint speeds, or whether the
  joint path between samples is smooth. Phase 2's optimiser is what owns those.
- The probe samples the rim only. Between samples the arm does whatever the
  planner decides, so a coarse sample count can certify a path the arm then
  cuts across — the chord error argument.
- `--orientation radial` is implemented and unit-tested but has not been
  exercised against the real workspace.
- The approach phase (getting to the circle's start, within the controller's
  2° splice guard) is Phase 2 and does not exist yet.

## Report-ready explanation

Point-to-point planning constrains only the endpoint, so the route between is
whatever the optimiser finds cheap. Tracing a shape requires constraining the
tool at every sample, which raises a prior question: can the arm hold the
requested pose all the way round? Answering it needs inverse kinematics at
each sample, seeded from its neighbour so that the solver does not jump
between the distinct joint configurations that reach the same pose. Doing this
exposed that the solver's convergence threshold, chosen when its only job was
to seed an optimiser that corrected the result afterwards, was two orders of
magnitude looser than the accuracy a traced path requires — so its residuals
described where the solver stopped rather than where the arm could reach.
Making that threshold a parameter of the request rather than a property of the
solver reduced the measured path error from 19.4 mm to 0.10 mm.

## Experiments that could support the thesis

- Traceable-radius envelope: for a grid of circle centres, the largest radius
  that stays fully on-path — a workspace map for path tasks rather than points.
- Residual against sample count, showing chord error falling quadratically.
- Fixed versus radial orientation policy over the same circles, quantifying
  what the orientation constraint costs in reachable workspace.
