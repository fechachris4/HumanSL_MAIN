# Panel Scene Editor Robotics Analysis Packet

## Physical objective
Represent a static exclusion volume rigidly attached to the supported torso-mounted rig, so GPMP2 plans its modelled arm spheres around it and the panel shows the same configured primitive. This is plan-time model evidence, not human-clearance proof.

## System decomposition
world_T_tcp = world_T_mount * mount_T_base * base_T_tcp(q)

The task changes only:
d_scene(p_mount) = min_i d_i(p_mount)

It does not change mount_T_base, base_T_tcp(q), world_T_mount, controller or actuation.

## Primitive equations
For a +Z cylinder centred at c, radius r, full height h:
q = [norm((p-c).xy) - r, abs((p-c).z) - h/2]
d_cylinder = norm(max(q, 0)) + min(max(q.x, q.y), 0)

For a mount-axis-aligned box centred at c with half extent b:
q = abs(p-c) - b
d_box = norm(max(q, 0)) + min(max(q.x, q.y, q.z), 0)

## Collision path
planner.yaml -> PlannerConfig.scene -> MakeMountSdf
-> SignedDistanceField -> ObstacleSDFFactorArm/GPArm
-> traced-path dense validation.

## Motion decomposition
Draft editing is local. Saving sends no command but changes future plan input. A later planner request may change motion, so persistence is blocked while commanding.

## Error decomposition
Configuration; frame convention; primitive distance; grid interpolation; GPMP2 residual; planned/executed redundant-posture divergence; physical torso/model mismatch.

## Time decomposition
Draft: browser-local. Save/SDF/GPMP2: non-real-time. SDF: rebuilt once per solve. Controller: unchanged 500 Hz consumer.

## Limiting cases
Empty/disabled scene; centre/side/cap/edge; non-positive dimension; upper grid face; overlapping objects; edit/save idle; edit/save commanding; direct filesystem edit; point plan without final SDF sweep.

## Falsifiable predictions
1. Empty scene reproduces the 10 m free field.
2. Cylinder centre distance is -min(r,h/2).
3. Side and cap surfaces have zero distance.
4. Multi-object field equals the minimum individual distance.
5. Rejected save leaves planner.yaml byte-for-byte unchanged.
6. Editing/dragging makes no HTTP request before SAVE.
7. SAVE makes only POST /api/scene.
8. Both arms report identical enabled scene identities.
9. Planned clearance remains planner-path evidence.

## A-E classification
Scene geometry in existing collision factors: C, planner behaviour.
Panel rendering/draft: A, diagnostic/editor only.
Save refusal: workflow boundary; no stop or command.

## Declared conventions and scope

The transform convention is `A_T_B`: the pose of frame B expressed in frame
A, mapping B-frame coordinates into A. `mount` is the root of
`GEN3_dual_mounted.urdf`, midway between the two base origins. All scene
centres and dimensions are finite values in metres in `mount`; no scene value
is expressed in an arm base or Vicon world frame. The cylinder centre is the
midpoint between its flat caps, its axis is mount +Z, and `height_m` is the
full cap-to-cap height. The box is axis-aligned in mount and stores positive
half-extents.

The static attachment is a planning-model assumption. The scene is neither
the separately tracked Vicon `Torso` segment nor proof that the physical
torso, support, clothing, cables or wearer remain inside the primitive. The
checked-in scene remains empty until Christian supplies or enters the physical
dimensions; no dimensions may be inferred from photographs, meshes or older
simulation assets.

The scene changes the cost and, for traced paths, the post-solve verdict of a
future plan. It adds no controller branch, command, automatic replan trigger,
warning or hard stop. GPMP2, YAML I/O, HTTP and drawing remain outside the
500 Hz execution path.

## Equation invariants and limiting-case derivation

For the cylinder, let `rho = norm((p-c).xy)` and `z = abs((p-c).z)`. The two
components of q are signed distances to the infinite side wall and to the
finite axial slab. Their standard intersection expression yields flat caps:

- at the centre, q is `[-r, -h/2]`, hence the distance is
  `-min(r, h/2)`;
- at `rho = r` with `z <= h/2`, the side distance is zero;
- at `z = h/2` with `rho <= r`, the cap distance is zero;
- at `rho = r, z = h/2`, the cap edge distance is zero;
- beyond both side and cap, the distance is the Euclidean norm of the radial
  and axial excesses.

This is not the older clamped-axis construction in
`TrajectoryGeneration/src/Obstacles.cpp`, which measures distance to the
closest point on an axis segment and therefore describes rounded capsule ends
after subtracting a radius. The new primitive must use the flat-capped formula
above. The box formula has the same inside/outside decomposition in three
axes. Non-positive, non-finite or shape-inapplicable dimensions are rejected
at the configuration boundary; they do not reach either formula. No arbitrary
`5.0 m` dimension rail is authorised: dimensions must be finite and strictly
positive, and each enabled primitive must fit the safely interpolatable grid.

For enabled primitives, the required field is
`d_scene(p) = min_i d_i(p)`. This minimum has the union's sign and zero set,
which is what collision factors require, but inside overlapping primitives it
is not always the exact Euclidean signed distance to the union boundary. An
enabled primitive participates in grid-bounds checking and the minimum. A
disabled primitive must still parse as complete, finite, positive geometry so
that enabling it cannot reveal malformed state, but it has no effect on
bounds or field construction; therefore a disabled out-of-grid primitive is
allowed. With no enabled primitive, every stored grid sample is exactly
`10.0 m`, preserving the existing free field. Overlap requires no special
branch: the field remains the minimum individual value.

The primitive equations are evaluated at SDF grid samples. GPMP2 queries the
stored field by trilinear interpolation, so the factor sees an interpolated
field between samples, not a fresh analytic evaluation. The current cell size
is 0.04 m. This discretisation is a named error contribution and no sub-cell
physical-clearance claim follows from an analytic sample test.

The range-check-admitted grid coordinates are
`[origin, origin + (n-1)*cell]`. The installed GPMP2 range check admits the
exact upper coordinate, but its trilinear interpolation then indexes the
next sample. The safely interpolatable interval includes the lower bound and
excludes that upper bound. Consequently every enabled primitive must satisfy
`primitive_min >= grid_min` and `primitive_max < grid_max` on every axis. The
upper comparison is deliberately strict; an object touching an upper face is
rejected before optimisation rather than clipped. Cylinder bounds are
`c +/- [r, r, h/2]`; box bounds are `c +/- b`.

## Factor and evidence semantics

Each arm model places its collision-sphere centres in mount. At support
states, `ObstacleSDFFactorArm` applies the field; between support states,
`ObstacleSDFFactorGPArm` applies it to GP-interpolated configurations. The
bundled GPMP2 factor adds each sphere radius and the configured
`epsilon_dist_m` to the hinge-loss threshold. `MakeMountSdf()` therefore
stores primitive distance only; it must not inflate the scene, and the panel
must not draw an implied epsilon boundary. `epsilon_dist_m` is an optimisation
cost activation distance, not a final hard clearance threshold.

Obstacle factors are optimisation residuals, not proof that the final path is
clear. Traced paths additionally sweep every modelled sphere over the final
dense joint trajectory, subtract the matching sphere radius from the queried
SDF distance, and reject penetration or an unanswerable SDF query. This sweep
is still planner-model evidence only: the controller receives world-Cartesian
pose/twist, not GPMP2's redundant joint posture, so executed joint posture and
clearance can differ.

Point-goal plans use the same SDF factors during optimisation but currently
perform only `ValidateJointPath()` after the solve; their summary explicitly
states that collision clearance is not computed. The scene-editor slice does
not close that final-sweep gap and must not describe a point plan as
independently clearance-validated.

Both arms must parse the same named scene from the same `planner.yaml` and
report the same ordered enabled identities. Their kinematics and collision
sphere sets remain arm-specific; identical scene identity does not imply
identical clearance or planned motion.

## Persistence, concurrency and UI boundary

The in-process planner receives the source-tree `planner.yaml` path and calls
`LoadPlannerConfig()` for each request. The existing reference state can issue
a later replan edge, including after recovery from prolonged world-data loss.
A successful panel save therefore can affect a later solve even though the
save itself invokes no solve. The server must reject `POST /api/scene` when
`session.status().commanding` is true, before writing, and every rejected
invalid, stale or commanding save must leave the file byte-for-byte unchanged.

This is a workflow boundary, not robot safety or repository-wide
immutability. A direct filesystem edit can bypass it. The session archive
copies `planner.yaml` at session start for provenance, but the current worker
still reads the live configured source path on a later solve. Freezing a
running session to the archive is deferred and must not be implied here.

Draft edits and handle drags update browser memory and drawing only. They send
no HTTP request. SAVE sends one whole-scene `POST /api/scene`; that endpoint
must not call preview solving, building, session start, runtime processes or
Kortex. An atomic successful replacement ensures a concurrent planner sees
either the old or new complete file. A stale source token prevents the panel
from overwriting a newer external edit.

The editor visibly distinguishes saved disk truth, unsaved draft and save
blocked while commanding. Rendering failure must not hide or disable the
existing stop path. The panel displays configured wireframe geometry only and
makes no SDF, collision, epsilon-inflation, clearance or human-safety
judgement.

## Error-to-evidence map

| Error source | Observable evidence | What the first slice can establish |
| --- | --- | --- |
| Configuration identity/type/value | strict parser error names the obstacle and field; planner digest and scene report | exact accepted YAML values and enabled identities |
| Mount/world/base mixing | scene types and renderer accept mount metres only; both arm reports echo one scene | no scene transform occurs after parsing |
| Primitive equation or height convention | hand-calculated centre, side, cap, edge and outside-corner samples | sample-level sign and magnitude correctness |
| Grid interpolation and coverage | exclusive-upper-face tests; GPMP2 query behavior; out-of-grid rejection | field is answerable over the declared volume, not sub-cell physical accuracy |
| GPMP2 residual/local optimum | support and interpolated factor diagnostics | obstacle cost was applied, not guaranteed clearance |
| Final planned collision | traced-path dense sphere sweep and `sdf_contents` | planned redundant posture clears modelled scene when the traced validator says so |
| Planned/executed posture divergence | future planner-vs-measured sphere overlays, kept separate | not established by this slice |
| Physical torso/model mismatch | rig measurement/calibration and supported-rig experiments | not established by YAML, UI, unit tests or simulation |

## Code-shape proposal

Scene values: PlannerConfig + StaticScene.h
Collision flow: PlanSolver.cpp -> MakeMountSdf()
Persistence: panel/scene_config.py
Drawing: scene.js + pure scene_geometry.mjs
500 Hz impact: none
Added concepts: named static scene; browser saved/draft pair
Removed concept: per-arm goal-box obstacle
No manager/service/registry/factory/plugin

`PlannerConfig` owns run-policy parsing, `StaticScene.h` owns Eigen-only value
types, `MakeMountSdf()` owns analytic-to-grid conversion, `scene_config.py`
owns text-preserving persistence and rejection, and the JavaScript modules own
drawing/draft arithmetic only. The completed migration must remove all
`goal.yaml box`, `--box`, `PlanRequest::obstacle` and legacy optional-box SDF
paths so no second obstacle definition survives.

## Source and independent checks

- `Christian_control/planning/src/MountSdf.cpp` currently establishes the
  AABB formula, exact `10.0` free samples and `(n-1)` grid maximum.
- `third_party/include/gpmp2/obstacle/SignedDistanceField.h` establishes the
  inclusive range check and eight-sample trilinear access that makes the exact
  upper face non-interpolatable.
- `third_party/include/gpmp2/obstacle/ObstacleCost.h` establishes that an
  out-of-range factor query returns zero obstacle cost.
- `third_party/include/gpmp2/obstacle/ObstacleSDFFactor-inl.h` and
  `ObstacleSDFFactorGP-inl.h` establish sphere-radius-plus-epsilon hinge loss
  at support and GP-interpolated states.
- `Christian_control/planning/src/PlanSolver.cpp` and
  `ValidatePath.cpp` establish per-solve SDF construction and the traced-path
  dense sphere sweep; `BridgeMain.cpp` explicitly records the missing
  point-plan clearance calculation.
- `Christian_control/planning/src/PlannerRuntime.cpp`,
  `Christian_control/runtime/InProcessPlanner.cpp`, and
  `Christian_control/runtime/Runner.cpp` establish live per-request planning
  outside the 500 Hz core.
- `Christian_control/panel/session.py` establishes that `commanding` means a
  controller process is alive, not merely that a session script exists.
- The GPMP2 IJRR paper, *Continuous-time Gaussian process motion planning via
  probabilistic inference* (Mukadam et al., 2018), independently describes a
  precomputed SDF over robot body points and obstacle factors at support and
  interpolated states. The bundled headers above remain the executable
  authority for this repository version.
- Direct hand evaluation for `r=0.20 m, h=0.60 m` produced cylinder distances
  `-0.20` at the centre, `0` at the side, cap and cap edge, and
  `sqrt(0.1^2 + 0.1^2) = 0.1414213562 m` at radial and axial excesses of
  `0.10 m`. An independent box evaluation produced zero at faces/edges/corners
  and Euclidean distance outside multiple axes.

## Adversarial analysis review

Verdict: **ACCEPTED**

Acceptance was supplied by the separate Task 0 adversarial reviewer, not by
the packet author. That reviewer checked all nine challenges below and the
2026-08-21 minimal-gating clarification, and returned ACCEPTED with no blocking
findings:

1. full `height_m` versus half-height use in equations, bounds, drawing and handles;
2. flat caps versus the older clamped-axis capsule behavior;
3. any mixing of mount, world or either arm-base frame;
4. the exclusive upper grid face and GPMP2 interpolation access;
5. parse-valid disabled objects versus their exclusion from bounds and field composition;
6. later automatic replans reading a saved live file despite SAVE issuing no solve;
7. surviving `goal.yaml`, `--box`, request or legacy SDF obstacle paths;
8. UI safety implications, including an always-usable stop path and no clearance claim;
9. the point-goal final SDF-sweep gap.

This reviewer-backed record releases the analysis gate for the approved
implementation slice only. It is not physical-clearance evidence and does not
authorise robot operation.
