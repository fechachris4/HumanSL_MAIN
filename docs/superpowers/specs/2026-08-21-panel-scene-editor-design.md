# Panel scene editor with one planner-owned obstacle definition

Date: 2026-08-21

Status: approved in chat by Christian on 2026-08-21. This document records the
approved architecture; it does not authorize implementation or robot operation.

## 1. Outcome

The existing panel gains a visual editor for the planner's static obstacle
scene. The operator can select an obstacle, change its primitive type,
mount-frame position and dimensions numerically or with handles in the 3D
viewer, preview unsaved changes immediately, and save the result to the same
YAML configuration the planner reads.

There is one persistent obstacle definition:

```text
planning/config/planner.yaml
```

The panel is an editor and visualizer of that file. It does not own a second
scene file, an obstacle database, an SDF implementation, clearance rules or a
robot-command path. `MakeMountSdf()` remains the boundary that converts the
planner-owned scene into the GPMP2 signed-distance field.

Editing or saving a scene must not move the physical robot. Draft edits are
browser-local. Saving never starts a session, invokes a solve, publishes a
trajectory, builds a binary or contacts Kortex. Because the running in-process
planner rereads live configuration on later solves, the panel refuses scene
saves while a controller is actively commanding either arm.

## 2. Scope

### 2.1 First implementation slice

The first slice provides:

- a shared static scene inside `planner.yaml`;
- named, enabled/disabled mount-frame obstacles;
- axis-aligned boxes and finite vertical cylinders;
- strict planner-side parsing and validation;
- composition of all enabled primitives in `MakeMountSdf()`;
- one panel scene editor using the existing mount-frame viewer;
- numeric editing and direct manipulation of position and dimensions;
- immediate unsaved preview;
- explicit reset and save actions;
- a server-side no-save-while-commanding gate;
- session provenance through the existing archived `planner.yaml` copy;
- focused hardware-free planner, panel and renderer tests.

### 2.2 Deferred overlays

The same viewer is intended to grow into a planning/execution diagnostic view,
but these are separate later slices:

- repair of the panel's stale `CART_TRAJ` plan-preview integration;
- planned trajectory display from the current world-Cartesian contract;
- executed trajectory display from recorded measured telemetry;
- planner-exported GPMP2 collision spheres along the planned joint path;
- planner-exported collision-sphere metadata applied to measured joint states;
- planned and executed clearance displays whose provenance is explicit.

The first slice must not create placeholders that imply these deferred
quantities are already measured or verified.

### 2.3 Explicitly out of scope

- tracked or moving obstacles;
- Vicon torso motion as obstacle input;
- arbitrary cylinder axes or obstacle rotations;
- capsules, meshes and general convex geometry;
- panel-side SDF, collision, clearance or safety calculations;
- SDF caching;
- automatic replanning after an edit or save;
- changing GPMP2 weights, controller behaviour or runtime safety policy;
- changing the pose/twist-only planner/controller contract;
- running or commanding either physical arm.

## 3. Physical and frame contract

The obstacle scene is a static approximation attached to the robot mount. It
models exclusion geometry for planning; it is not an anatomical reconstruction
and is not the separately tracked `Torso` Vicon segment.

The planner's internal frame remains `mount`, the root of
`GEN3_dual_mounted.urdf` at the midpoint of the two base origins. Every scene
coordinate is expressed in metres in this frame. No scene primitive carries an
implicit arm-base or world frame.

The first cylinder convention is:

```text
centre_mount_m       midpoint between the two flat caps
axis                 mount +Z
radius_m             radial distance to the side wall
height_m             full cap-to-cap distance
```

The first box convention is:

```text
centre_mount_m       centre of the box
half_extent_m        positive half-size along mount X, Y and Z
orientation          axis-aligned in mount
```

All dimensions must be finite and strictly positive. Centres must be finite.
Every enabled primitive must fit wholly within the interpolatable SDF volume;
an out-of-grid object is rejected before optimization rather than clipped.

This task changes plan-time modelled collision geometry. It does not prove
physical torso clearance, wearer safety, executed clearance or equivalence
between GPMP2's planned redundant posture and the controller's executed
posture.

## 4. Persistent YAML schema

`planner.yaml` remains the one run-policy file and gains a named scene under
the existing `obstacles` section:

```yaml
obstacles:
  epsilon_dist_m: 0.05
  collision_sigma: 0.0005

  scene:
    torso:
      enabled: true
      shape: cylinder
      center_mount_m: [0.0, 0.0, 0.25]
      radius_m: 0.22
      height_m: 0.60

    bench:
      enabled: false
      shape: box
      center_mount_m: [0.45, 0.0, -0.15]
      half_extent_m: [0.30, 0.50, 0.10]
```

The mapping key (`torso`, `bench`) is the stable obstacle identity shown by
the panel and diagnostics. Names must be non-empty and unique by YAML mapping
construction. The parser accepts only `box` and `cylinder` in the first slice
and requires exactly the keys belonging to the selected shape. Unknown,
missing, non-finite or wrongly typed values fail with the obstacle identity in
the error.

The checked-in default remains an empty scene until Christian supplies the
torso dimensions and centre or enters them through the editor. No physical
dimension is inferred from photographs, URDF meshes or old simulation assets,
and no plausible-looking placeholder is committed as though it described the
rig.

The per-arm optional `box:` in `goal.yaml` and the bridge's `--box` path are
retired in the same migration. Leaving either active would preserve two
obstacle-definition mechanisms and violate the outcome. The checked-in box
examples are currently comments, so this migration removes no active obstacle
from the default configuration.

## 5. Planner ownership

### 5.1 Plain data types

The planner owns small value types rather than a scene manager:

```cpp
struct MountCylinder {
    Eigen::Vector3d center_mount_m;
    double radius_m;
    double height_m;
};

using StaticObstacle = std::variant<AxisAlignedBox, MountCylinder>;

struct NamedStaticObstacle {
    std::string id;
    bool enabled;
    StaticObstacle geometry;
};
```

`PlannerConfig` carries the parsed scene as run policy. The parsed values are
the only obstacle geometry passed into solving.

### 5.2 SDF construction

`MakeMountSdf()` accepts the complete scene. At every grid sample it evaluates
the exact analytic signed distance of each enabled primitive and stores their
minimum:

```text
d_scene(p) = min_i d_i(p)
```

No enabled objects means the existing large uniform free distance. The box
formula remains the standard AABB SDF. The cylinder formula represents a
finite flat-capped cylinder, not the old frozen tree's clamped-axis capsule.

Nothing below this boundary changes:

```text
planner.yaml scene
  -> strict PlannerConfig parsing
  -> MakeMountSdf
  -> gpmp2::SignedDistanceField
  -> ObstacleSDFFactorArm at support states
  -> ObstacleSDFFactorGPArm between support states
  -> existing dense traced-path collision validation
```

Point-goal plans still lack the traced path's independent final SDF sweep.
That limitation must remain explicit in the first-slice report and UI. A later
validator-unification task may close it, but it is not silently bundled into
the scene editor.

### 5.3 No first-slice cache

The SDF remains local to and rebuilt once per solve. A mount-fixed scene makes
caching possible, but introducing cache identity, invalidation and lifetime is
not necessary to establish the correct ownership path. It is deferred until
profiling shows the rebuild matters.

## 6. Panel server boundary

The panel server gains a small scene module responsible only for YAML text
round-tripping and early input validation. It uses `paths.PLANNER_YAML` and the
existing `yaml_text` replacement approach so unrelated keys and comments
survive.

The HTTP surface is:

```text
GET  /api/scene
POST /api/scene
```

`GET /api/scene` returns:

- source path;
- parsed named obstacle values;
- source-file change token or digest;
- parse error, if disk truth is malformed;
- whether saving is currently allowed;
- the reason saving is blocked when a controller is commanding.

`POST /api/scene` accepts the complete intended named scene plus the change
token it was edited from. It:

1. rejects when `session.status().commanding` is true;
2. rejects stale writes when the file changed after the editor loaded it;
3. validates names, shapes, finite values and positive dimensions;
4. rewrites only `obstacles.scene` in a temporary sibling file;
5. validates the resulting complete planner configuration with the panel's
   strict structural/type checks; the C++ `LoadPlannerConfig()` parser remains
   the final authority when a solve reads the saved file;
6. atomically replaces `planner.yaml` only after validation succeeds;
7. returns the reread disk truth.

No partial field endpoint is needed. A whole-scene write lets deletion,
addition, renaming and shape changes be one atomic edit while keeping the rest
of `planner.yaml` text intact. The existing one-time `.panel.bak` convention is
preserved.

The endpoint never calls the preview solver, session launcher, build system,
runtime process or Kortex API.

## 7. Why saving is blocked during commanded motion

The production controller holds paths to the source-tree `planner.yaml` in
`PlannerRuntimeConfig`. `SolvePlanForRequest()` invokes `SolvePlan()`, which
loads that file for each planning request. A later automatic replan can
therefore observe a scene saved after the session began.

The causal path is:

```text
panel saves live planner.yaml
  -> later replan request
  -> LoadPlannerConfig reads the changed scene
  -> new GPMP2 result
  -> controller may activate a different trajectory
```

Consequently, a save button that merely avoids invoking `/api/plan/solve` is
not sufficient. The server-side command-state gate makes the panel guarantee
real: visual draft editing remains available, but persistence waits until no
controller is commanding.

This is a panel workflow boundary, not a claim that direct filesystem edits
are impossible. Freezing the production planner onto its session-archived
configuration would provide repository-wide immutability, but requires a
separate runtime configuration design and is deferred.

## 8. Viewer and editor interaction

The existing Scene card and `scene.js` remain the one renderer implementation.
It changes from one `obstacle` value to an ordered collection of named scene
objects. Rendering consumes the same values returned from `/api/scene` and
performs no frame conversion because the schema is mount-only.

An `EDIT SCENE` action opens an inspector beside the existing canvas. It does
not create a second viewer or persistent client model. The editor contains:

- obstacle selector by stable name;
- add, rename and delete actions;
- enabled checkbox;
- shape selector;
- X/Y/Z centre fields, metres in mount;
- radius and height for a cylinder;
- X/Y/Z half-extents for a box;
- `RESET TO SAVED`;
- `SAVE SCENE`.

Every valid field edit updates a browser-local draft and redraws immediately.
Invalid or incomplete fields keep the last drawable draft and show the exact
field error; they never become zero and never reach disk.

Direct manipulation uses explicit handles so object editing and camera motion
cannot be confused:

- dragging empty canvas orbits the camera, as today;
- the centre/plane handle changes mount X/Y;
- the vertical handle changes mount Z;
- the cylinder radial handle changes radius;
- the cylinder cap handle changes height;
- the box axis handles change the corresponding half-extent.

Dragging updates the same numeric draft fields. Dimensions are clamped only to
a small positive UI floor needed to keep a handle usable; the planner parser
remains the authority that accepts or rejects saved values.

The editor always displays one of these states:

```text
SAVED — planner disk truth
UNSAVED DRAFT — not used by planner
SAVE BLOCKED — controller is commanding
```

The viewer draws wireframe primitives for visual comprehension. It does not
draw an inflated epsilon boundary in the first slice, because doing so would
require panel-side collision semantics. The configured GPMP2 epsilon remains a
separate numeric planner setting.

## 9. Robot display in the first slice

The first slice reuses the existing two-arm DH skeleton and live
measured/commanded joint state. It does not label that skeleton as collision
geometry.

The existing joint DH tables are generated from the canonical URDF. The
mount-to-base transforms are currently copied as JavaScript defaults. As part
of this slice, `/api/dh` should return the mount transforms read from
`model/dual_arm_mounting.yaml`, and the viewer should use those returned
values. This removes an existing copied robot-placement constant while the
scene is being made frame-auditable.

No URDF mesh loader or Three.js/WebGL dependency is introduced. The current
canvas renderer is adequate for two primitives, handles and line-art robot
context. A rendering-library migration would be a separate product decision,
not a prerequisite for correct scene ownership.

## 10. Later diagnostic overlays without duplicated geometry

### 10.1 Planned trajectory

The current panel preview still expects the retired joint `TRAJ_BEGIN` format
and omits arguments required by the current bridge. A later slice repairs it
to parse `CART_TRAJ_BEGIN` and provide explicit preview provenance. The viewer
then draws the returned Cartesian positions directly.

GPMP2's internal planned joint posture remains planner-only. If a collision-
sphere trajectory is needed, it is exported as a diagnostic artifact; it is
not added to the controller contract.

### 10.2 Executed trajectory

Executed position comes from recorded measured telemetry. It is transformed
to mount with the recorded `world_T_mount(t)` associated with the same sample,
never with a current or guessed Mount pose. Live and replay paths use the same
telemetry interpretation.

### 10.3 Collision spheres

The panel must not copy the sphere stations or radii from
`GenerateArmModel.cpp`. A hardware-free planner diagnostic boundary will
export sphere identity, radius and mount-frame centre for requested joint
states using the real `PlannerModel`/`gpmp2::ArmModel`. The viewer only draws
that output.

This preserves the distinction:

```text
planned spheres    GPMP2 internal q(t), planner-path evidence
executed spheres   measured q(t), execution evidence under the model
```

Neither is labelled physical wearer clearance without the corresponding
calibration and physical evidence.

## 11. Error and concurrency behaviour

- Missing or malformed `planner.yaml`: show disk error; do not fabricate an
  empty scene and do not allow overwrite until explicitly reloaded/repaired.
- Browser draft invalid: show field error; disk unchanged.
- Concurrent filesystem edit: reject stale save and reload disk truth only on
  operator request.
- Controller commanding: reject save; draft remains visible and recoverable.
- Save failure: retain draft; state clearly that planner disk truth is
  unchanged.
- Successful save: reread the file and replace draft with returned disk truth.
- Panel refresh or crash: unsaved draft is lost; no robot or planner state is
  changed.
- GPMP2 solve failure after a saved scene: planner reports its own failure;
  panel does not reinterpret it.

## 12. Verification

### 12.1 Planner tests

- strict parsing of an empty scene, one cylinder, one box and multiple named
  objects;
- unknown shape, missing key, extra key, non-finite centre, non-positive
  dimension and out-of-grid rejection;
- analytic cylinder SDF at centre, side wall, above/below caps, cap edge and
  outside corner;
- box regression against the existing analytic SDF;
- minimum-distance composition for overlapping and separated primitives;
- disabled objects have no effect;
- both arm solves receive the same mount scene;
- the prior no-obstacle behavior remains uniformly free.

### 12.2 Panel server tests

- read exact YAML values and stable names;
- whole-scene write preserves unrelated tuning and comments;
- add, rename, delete and shape change round-trip;
- invalid writes leave the file byte-for-byte unchanged;
- stale change token rejects without overwrite;
- `commanding=true` rejects without overwrite;
- successful write uses atomic replacement and returns reread disk truth;
- scene endpoints never invoke solve, build, session start or a robot-facing
  process.

### 12.3 Renderer/editor tests

- cylinder and box coordinates are interpreted in mount metres;
- numeric changes update only draft state;
- handle drag and numeric entry produce the same draft values;
- empty-canvas drag still orbits;
- invalid input never becomes zero;
- saved/draft/blocked state labels follow the actual state;
- multiple obstacles render and selection affects only one;
- the stop-button path remains usable if scene rendering throws.

The removed pre-migration panel tests are not resurrected wholesale. A focused
current test surface is added for the modules changed by this task.

### 12.4 Independent evidence

Tests added with the implementation are not the sole oracle. Verification also
compares:

- parsed mount transforms against `dual_arm_mounting.yaml` and the existing
  URDF parity test;
- SDF sample values against hand-calculated primitive distances;
- the scene echoed in planner diagnostics against the saved YAML;
- a solved path's validator-reported `sdf_contents` against the named enabled
  objects;
- session-archived `planner.yaml` against the file used at session start.

All verification is hardware-free. No compilation, panel preview or simulation
authorizes a physical-arm run.

## 13. Expected implementation shape

The anticipated production touch surface is deliberately small:

```text
planning/config/planner.yaml
planning/src/PlannerConfig.{h,cpp}
planning/src/MountSdf.{h,cpp}
planning/src/PlanSolver.{h,cpp}
planning/src/BridgeMain.cpp          retire per-goal/CLI box plumbing
panel/paths.py
panel/planner_config.py or a small panel/scene_config.py
panel/server.py
panel/dh.py
panel/static/index.html
panel/static/panel.css
panel/static/panel.js
panel/static/scene.js
focused planner and panel tests
```

One new panel module is acceptable if it replaces scene-specific parsing and
writing that would otherwise spread through `planner_config.py` and
`server.py`. No manager, service, registry, plugin system or generic geometry
framework is introduced.

Concept budget for the first slice:

- add one planner scene value (`NamedStaticObstacle` plus its two primitive
  alternatives);
- add one panel scene editor state (`saved` plus browser-local `draft`);
- add one save boundary with command-state and stale-write checks;
- remove the per-arm goal-box obstacle path;
- add no controller concept, control branch or hardware safety threshold.

Unexpected spread into control mathematics, Kortex lifecycle, the 500 Hz core
or the planner/controller contract stops implementation for review.

## 14. Alternatives considered

### A. Scene inside `planner.yaml` — adopted

This reuses the existing strict planner config, panel edit path, source-tree
runtime path, digest reporting and session archive. It is the smallest route
to one persistent definition.

### B. Separate `scene.yaml` — dismissed for this slice

It separates tuning from geometry, but adds a new runtime path, panel path,
archive artifact, digest and failure mode before scene size justifies it. It
may be reconsidered if the scene becomes large or independently reusable.

### C. Invoke a planner executable for every visual edit — dismissed

This centralizes parsing but creates a process/serialization boundary on the
drag path and is too heavy for immediate interaction. Planner-owned diagnostic
export remains the recommended later route for collision spheres, where exact
model parity matters and update rate is lower.

### D. Freeze a running session onto its archived planner config — deferred

This would make active-session configuration immutable even against direct
filesystem edits. It is stronger than the panel save gate but changes runtime
configuration ownership and command-line contracts. It deserves its own
design if repository-wide session immutability becomes a requirement.

## 15. Acceptance criteria

The first slice is complete when:

1. One YAML scene supplies both the panel view and `MakeMountSdf()`.
2. No active per-arm goal obstacle path remains.
3. A torso cylinder can be selected, repositioned and resized numerically or
   with viewer handles in mount coordinates.
4. Unsaved changes redraw immediately and are visibly labelled as drafts.
5. Saving writes the scene back to `planner.yaml` without regenerating the
   rest of the file.
6. Saving neither solves nor sends any command.
7. Saving is refused server-side while a controller is commanding.
8. The next offline solve reads the saved scene and reports the enabled named
   objects in its SDF contents.
9. The displayed primitive uses the exact persisted centre and dimensions;
   the panel performs no collision inflation or clearance decision.
10. Both robot arms are shown in the same mount frame using generated DH and
    served mount transforms.
11. All focused hardware-free tests pass.
12. No robot-facing executable is run.
