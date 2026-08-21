# Planner-Owned Panel Scene Editor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a mount-frame static-obstacle editor to the existing panel while keeping `planning/config/planner.yaml` as the only persistent scene definition and `MakeMountSdf()` as the only planner collision conversion.

**Architecture:** `PlannerConfig` strictly parses named box/cylinder values from `planner.yaml` into plain C++ scene values, and `MakeMountSdf()` composes their analytic signed distances for both arms. The panel reads and atomically replaces only the YAML scene block, holds unsaved edits in browser memory, renders those values without collision calculations, and refuses persistence while a controller is commanding.

**Tech Stack:** C++17, Eigen, yaml-cpp, GTSAM/GPMP2, CMake/CTest, Python 3.12 standard library, pytest 7.4, browser-native ES modules/canvas, Node 22 built-in test runner.

**Spec:** `docs/superpowers/specs/2026-08-21-panel-scene-editor-design.md`

## Global Constraints

- Read the spec, `docs/engineering/humansl-engineering-contract.md`, `docs/engineering/robotics-analysis-workflow.md`, `docs/intent/story.md`, and current overlapping files before editing.
- This is a Level 2 planning-model change. Task 0's Robotics Analysis Packet and adversarial acceptance gate all production edits.
- Frame convention is `A_T_B`; every scene position and dimension is in metres in the URDF `mount` frame.
- Cylinders are finite, flat-capped, centred at `center_mount_m`, aligned with mount `+Z`, and use full `height_m`.
- Boxes are axis-aligned in mount and use positive `half_extent_m`.
- Do not infer or commit torso dimensions from photographs, old simulations, URDF meshes or plausible defaults. The checked-in scene stays empty until Christian supplies values or enters them through the editor.
- `planner.yaml` is the sole persistent scene. Retire the per-arm `goal.yaml` box and `--box`; do not leave compatibility aliases.
- The panel performs no SDF, collision, epsilon inflation, clearance decision, automatic solve/replan, build or robot command on edit/save.
- Browser drafts may change at any time. `POST /api/scene` rejects while `session.status().commanding` is true and leaves disk unchanged.
- Keep GPMP2, file I/O, panel work and rendering outside the 500 Hz thread. Do not touch controller mathematics, Kortex lifecycle or the Cartesian handoff contract.
- Point-goal final clearance remains an explicit limitation. SDF caching remains deferred.
- Preserve unrelated user changes, especially `CLAUDE.md` and `docs/intent/raw-prompt-log.md`.
- No robot-facing executable may be run. All verification is hardware-free and does not authorize hardware motion.
- Every production task is test-first and ends with a focused commit. Use `superpowers:verification-before-completion` and `superpowers:requesting-code-review` at the final gate.

## File Structure and Ownership

**Create**

- `docs/engineering/analysis/2026-08-21-panel-scene-editor-analysis.md` — accepted Robotics Analysis Packet.
- `Christian_control/planning/src/StaticScene.h` — Eigen-only scene values; no YAML or GPMP2.
- `Christian_control/planning/tests/test_scene_config.cpp` — strict YAML scene parsing.
- `Christian_control/planning/tests/test_bridge_args.cpp` — retired `--box` rejection.
- `Christian_control/panel/scene_config.py` — scene read/write, stale-write and command-state gates.
- `Christian_control/panel/tests/test_yaml_text.py`, `test_scene_config.py`, `test_dh.py`, `test_plan.py`, `test_static.py` — focused restored panel tests.
- `Christian_control/panel/static/scene_geometry.mjs` — pure display geometry/edit deltas, never collision.
- `Christian_control/panel/static/scene_geometry.test.mjs` — Node tests.

**Modify**

- Planner: `config/planner.yaml`, `config/goal.yaml`, `src/PlannerConfig.{h,cpp}`, `src/MountSdf.{h,cpp}`, `src/PlanSolver.{h,cpp}`, `src/BridgeMain.cpp`, `tests/test_mount_sdf.cpp`, `CMakeLists.txt`.
- Panel backend: `paths.py`, `yaml_text.py`, `plan.py`, `planner_config.py`, `dh.py`, `server.py`.
- Panel UI: `static/index.html`, `panel.css`, `panel.js`, `scene.js`, `scene.test.html`.
- Verified docs: `Christian_control/README.md` and `docs/architecture.md`.

---

### Task 0: Robotics Analysis and Code-Shape Gate

**Files:**
- Create: `docs/engineering/analysis/2026-08-21-panel-scene-editor-analysis.md`
- Read: approved spec and all Global Constraints sources.

**Interfaces:**
- Consumes: approved design and current mount-internal collision flow.
- Produces: accepted equations/invariants for every later edit.

- [ ] **Step 1: Write the full analysis packet before production code**

Include these exact sections and content:

```markdown
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
```

- [ ] **Step 2: Record the code-shape proposal**

```markdown
Scene values: PlannerConfig + StaticScene.h
Collision flow: PlanSolver.cpp -> MakeMountSdf()
Persistence: panel/scene_config.py
Drawing: scene.js + pure scene_geometry.mjs
500 Hz impact: none
Added concepts: named static scene; browser saved/draft pair
Removed concept: per-arm goal-box obstacle
No manager/service/registry/factory/plugin
```

- [ ] **Step 3: Obtain adversarial read-only acceptance**

The reviewer must challenge height versus half-height, capsule versus flat caps, mount/world/base mixing, grid upper-face interpolation, disabled-object rules, later automatic replans, surviving duplicate box paths, UI safety implications and the point-plan validation gap. Append `ACCEPTED` with resolved findings or `REJECTED`. Stop on rejection.

- [ ] **Step 4: Commit the gate**

```bash
git add docs/engineering/analysis/2026-08-21-panel-scene-editor-analysis.md
git commit -m "docs: analyze planner-owned panel scene"
```

---

### Task 1: Canonical Scene Types and Strict YAML Parsing

**Files:**
- Create: `Christian_control/planning/src/StaticScene.h`
- Create: `Christian_control/planning/tests/test_scene_config.cpp`
- Modify: `Christian_control/planning/src/PlannerConfig.h`
- Modify: `Christian_control/planning/src/PlannerConfig.cpp`
- Modify: `Christian_control/planning/config/planner.yaml`
- Modify: `Christian_control/planning/CMakeLists.txt`

**Interfaces:**
- Produces:

```cpp
struct AxisAlignedBox {
    // metres, mount frame. Existing member spellings are retained until
    // the old request path is removed, so every intermediate commit builds.
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d half_extent = Eigen::Vector3d::Zero();
};
struct MountCylinder {
    Eigen::Vector3d center_mount_m = Eigen::Vector3d::Zero();
    double radius_m = 0.0;
    double height_m = 0.0;
};
using StaticObstacleGeometry = std::variant<AxisAlignedBox, MountCylinder>;
struct NamedStaticObstacle {
    std::string id;
    bool enabled = false;
    StaticObstacleGeometry geometry;
};
const char* StaticObstacleShapeName(const StaticObstacleGeometry&);
```

`PlannerConfig` adds `std::vector<NamedStaticObstacle> scene;`.

- [ ] **Step 1: Write failing complete-config parser tests**

Use a helper that writes every required planner section plus a supplied `obstacles.scene`. Assert:

```cpp
const PlannerConfig empty = LoadPlannerConfig(WriteConfig("scene: {}"));
Check(empty.scene.empty(), "empty scene parses");

const PlannerConfig config = LoadPlannerConfig(WriteConfig(R"(
scene:
  torso:
    enabled: true
    shape: cylinder
    center_mount_m: [0.1, -0.2, 0.3]
    radius_m: 0.22
    height_m: 0.60
)"));
Check(config.scene.size() == 1, "one cylinder");
Check(config.scene[0].id == "torso", "mapping key is identity");
const auto& torso = std::get<MountCylinder>(config.scene[0].geometry);
Check(Near(torso.center_mount_m, Eigen::Vector3d(0.1, -0.2, 0.3)),
      "mount centre");
Check(torso.radius_m == 0.22 && torso.height_m == 0.60, "radius/full height");
```

Also reject unknown shape, missing/extra keys, non-map scene, non-boolean enabled, non-finite centre and non-positive dimensions. Parse a disabled but numerically valid object without dropping it.

- [ ] **Step 2: Run and observe the expected failure**

```bash
cmake -S Christian_control/planning -B Christian_control/planning/build
cmake --build Christian_control/planning/build --target test_scene_config -j2
```

Expected: target/types absent or `obstacles.scene` rejected as unknown.

- [ ] **Step 3: Implement the value header and strict parser**

Set the checked-in YAML to:

```yaml
obstacles:
  epsilon_dist_m: 0.05
  collision_sigma: 0.0005
  scene: {}
```

Require exactly `epsilon_dist_m`, `collision_sigma` and `scene`. Require scene to be a mapping. For each ID, read `enabled` and `shape`, then exact keys:

- cylinder: `enabled, shape, center_mount_m, radius_m, height_m`
- box: `enabled, shape, center_mount_m, half_extent_m`

Reuse `ReadVector3`. Require dimensions in `(0, 5.0]` m as a typo rail, not a physical limit. Add no geometry defaults.

- [ ] **Step 4: Report persisted scene values**

`EffectiveConfigText` prints count, stable ID, enabled state, shape, centre and shape dimensions. It calculates no clearance.

- [ ] **Step 5: Run focused evidence**

```bash
cmake --build Christian_control/planning/build --target test_scene_config test_mount_sdf -j2
ctest --test-dir Christian_control/planning/build -R '^(scene_config|mount_sdf)$' --output-on-failure
git diff --check
```

- [ ] **Step 6: Commit**

```bash
git add Christian_control/planning/src/StaticScene.h \
  Christian_control/planning/src/PlannerConfig.h \
  Christian_control/planning/src/PlannerConfig.cpp \
  Christian_control/planning/config/planner.yaml \
  Christian_control/planning/tests/test_scene_config.cpp \
  Christian_control/planning/CMakeLists.txt
git commit -m "planner: parse one mount-frame obstacle scene"
```

---

### Task 2: Compose Box and Flat-Capped Cylinder SDFs

**Files:**
- Modify: `Christian_control/planning/src/MountSdf.h/.cpp`
- Modify: `Christian_control/planning/tests/test_mount_sdf.cpp`

**Interfaces:**
- Produces:

```cpp
gpmp2::SignedDistanceField MakeMountSdf(
    const GridGeometry&,
    const std::vector<NamedStaticObstacle>& scene_mount);
// Migration-only overload removed in Task 3 after callers switch:
gpmp2::SignedDistanceField MakeMountSdf(
    const GridGeometry&, const std::optional<AxisAlignedBox>&);
GridBounds StaticObstacleBounds(const StaticObstacleGeometry&);
bool StaticObstacleWithinGridBounds(
    const StaticObstacleGeometry&, const GridBounds&);
std::string DescribeStaticScene(
    const std::vector<NamedStaticObstacle>&, const GridGeometry&);
```

- [ ] **Step 1: Add independent failing SDF tests**

Use a deliberately simple test grid with origin `[-0.4,-0.4,-0.4]`, cell
`0.1`, and `10×10×10` samples, so every query below lands exactly on a stored
sample and trilinear interpolation cannot hide a wrong primitive formula.
Calculate expected values independently in the test:

```cpp
MountCylinder cylinder;
cylinder.center_mount_m = Eigen::Vector3d::Zero();
cylinder.radius_m = 0.20;
cylinder.height_m = 0.60;
NamedStaticObstacle named{"test-cylinder", true, cylinder};
const auto sdf = MakeMountSdf(test_grid, {named});
const auto distance_at = [&sdf](double x, double y, double z) {
    return sdf.getSignedDistance(gtsam::Point3(x, y, z));
};
CheckNear(distance_at(0, 0, 0), -0.20);
CheckNear(distance_at(0.20, 0, 0), 0.0);
CheckNear(distance_at(0, 0, 0.30), 0.0);
CheckNear(distance_at(0.20, 0, 0.30), 0.0);
CheckNear(distance_at(0.30, 0, 0.40),
          std::sqrt(0.10 * 0.10 + 0.10 * 0.10));
```

Also test existing box values, empty scene = `10.0`, disabled object ignored, two objects = minimum distance, enabled out-of-grid throws with ID, exclusive upper face rejects, disabled out-of-grid object does not affect construction.

- [ ] **Step 2: Run and observe signature/formula failure**

```bash
cmake --build Christian_control/planning/build --target test_mount_sdf -j2
```

- [ ] **Step 3: Implement exact primitive distances**

```cpp
double CylinderSignedDistance(const Eigen::Vector3d& p,
                              const MountCylinder& cylinder) {
    const Eigen::Vector3d local = p - cylinder.center_mount_m;
    const Eigen::Vector2d q(
        local.head<2>().norm() - cylinder.radius_m,
        std::abs(local.z()) - 0.5 * cylinder.height_m);
    return q.cwiseMax(0.0).norm() + std::min(q.maxCoeff(), 0.0);
}
```

At each sample, start at `10.0`, skip disabled objects and take the minimum primitive distance. Bounds are centre ± `[r,r,h/2]` for cylinders and centre ± half extent for boxes. Preserve the strict upper grid comparison.

- [ ] **Step 4: Preserve a compiling migration seam**

Implement the new scene overload as the real implementation. Keep the old
optional-box overload for one commit only; it wraps the box as one enabled
`NamedStaticObstacle{"legacy-box", true, *box}` or passes an empty scene.
Task 3 switches every caller and deletes this overload, so it never survives
the completed migration.

- [ ] **Step 5: Verify focused tests**

```bash
cmake --build Christian_control/planning/build --target \
  test_mount_sdf planner_bridge -j2
ctest --test-dir Christian_control/planning/build \
  -R '^mount_sdf$' --output-on-failure
git diff --check
```

- [ ] **Step 6: Commit the primitive/SDF implementation**

```bash
git add Christian_control/planning/src/MountSdf.h \
  Christian_control/planning/src/MountSdf.cpp \
  Christian_control/planning/tests/test_mount_sdf.cpp
git commit -m "planner: build mount SDF from named static scene"
```

---

### Task 3: Retire the Per-Arm Goal Box and CLI Override

**Files:**
- Create: `Christian_control/planning/tests/test_bridge_args.cpp`
- Create: `Christian_control/panel/tests/test_plan.py`
- Modify: `Christian_control/planning/src/BridgeMain.cpp`
- Modify: `Christian_control/planning/src/PlanSolver.h/.cpp`
- Modify: `Christian_control/planning/config/goal.yaml`
- Modify: `Christian_control/planning/CMakeLists.txt`
- Modify: `Christian_control/panel/plan.py`
- Modify: `Christian_control/panel/static/panel.js`

**Interfaces:**
- Produces: goals/paths only; `--box` unrecognized; active `goal.yaml box` rejected with migration guidance.
- Consumes: `PlannerConfig::scene` and the new scene-based `MakeMountSdf`.

- [ ] **Step 1: Add failing retirement tests**

C++:

```cpp
std::ostringstream targets, diagnostics;
const int code = RunBridge(
    {"--box", "0", "0", "0", "1", "1", "1"},
    targets, diagnostics);
Check(code == 1, "retired flag rejected");
Check(targets.str().empty(), "no trajectory emitted");
Check(diagnostics.str().find("unrecognized flag: '--box'") != std::string::npos,
      "diagnostic names flag");
```

Python:

```python
def test_goal_box_is_rejected_as_retired():
    text = """session_arms: right
right:
  frame: mount
  goal: [0.5, 0.0, 0.4]
  box:
    center: [0.0, 0.0, 0.0]
    half_extent: [0.1, 0.1, 0.1]
"""
    assert plan.validate_goal(text) == [
        "right: box is retired — edit obstacles.scene in planner.yaml"
    ]
```

Also require `write_goal_fields` to reject a payload containing `fields["box"]` rather than ignore it.

- [ ] **Step 2: Run and confirm old support fails**

```bash
cmake --build Christian_control/planning/build --target test_bridge_args -j2
python3 -m pytest Christian_control/panel/tests/test_plan.py -q
```

- [ ] **Step 3: Switch both solvers, then remove all old plumbing**

Use in both solver paths:

```cpp
const auto sdf = MakeMountSdf(MountGridGeometry(), config.scene);
```

Remove `PlanRequest::obstacle`, the `SolveAlongPath` obstacle parameter, and
the migration-only optional-box `MakeMountSdf` overload.

Remove from `BridgeMain.cpp`: usage/parser `--box`, `DeclaredBox`, parsed box, goal-box parsing, frame conversion/inflation, box bounds helpers and solver arguments. Reject active goal box with:

```cpp
if (arm_node["box"])
    throw std::invalid_argument(
        "box is retired — edit obstacles.scene in planner.yaml");
```

Remove box examples from `goal.yaml` and box controls/submission from `plan.py/panel.js`. Raw goal validation rejects the retired key.

- [ ] **Step 4: Prove one obstacle source**

```bash
rg -n "DeclaredBox|--box|block\\.box|box-on|box-center|box-half|request\\.obstacle" \
  Christian_control/planning Christian_control/panel \
  --glob '!**/build/**' --glob '!**/*.bak'
```

Expected: only tests/migration messages; review every match.

- [ ] **Step 5: Run and commit**

```bash
cmake --build Christian_control/planning/build --target test_bridge_args planner_bridge -j2
ctest --test-dir Christian_control/planning/build -R '^bridge_args$' --output-on-failure
python3 -m pytest Christian_control/panel/tests/test_plan.py -q
git diff --check
git add Christian_control/planning/src/BridgeMain.cpp \
  Christian_control/planning/src/PlanSolver.h \
  Christian_control/planning/src/PlanSolver.cpp \
  Christian_control/planning/src/MountSdf.h \
  Christian_control/planning/src/MountSdf.cpp \
  Christian_control/planning/config/goal.yaml \
  Christian_control/planning/tests/test_bridge_args.cpp \
  Christian_control/planning/CMakeLists.txt \
  Christian_control/panel/plan.py \
  Christian_control/panel/static/panel.js \
  Christian_control/panel/tests/test_plan.py
git commit -m "planner: retire per-goal obstacle boxes"
```

---

### Task 4: Panel Scene Persistence and Save Gate

**Files:**
- Create: `Christian_control/panel/scene_config.py`
- Create: `Christian_control/panel/tests/test_yaml_text.py`
- Create: `Christian_control/panel/tests/test_scene_config.py`
- Modify: `Christian_control/panel/yaml_text.py`
- Modify: `Christian_control/panel/planner_config.py`
- Modify: `Christian_control/panel/server.py`

**Interfaces:**

```python
def read_scene(path: Path | None = None) -> dict[str, Any]: ...

def write_scene(
    submitted: dict[str, Any],
    source_fnv1a64: int,
    *,
    path: Path | None = None,
    commanding: Callable[[], bool],
) -> tuple[bool, dict[str, Any] | str]: ...

def replace_block(text: str, path: tuple[str, ...],
                  rendered_block: str) -> str | None: ...
```

HTTP is exactly `GET /api/scene` and `POST /api/scene`.

- [ ] **Step 1: Add failing block-replacement tests**

```python
def test_replace_block_preserves_unrelated_text_byte_for_byte():
    original = """# header
obstacles:
  epsilon_dist_m: 0.05  # keep this
  scene: {}
solver:
  max_iterations: 1000
"""
    changed = yaml_text.replace_block(
        original, ("obstacles", "scene"),
        "scene:\n    torso:\n      enabled: true")
    assert "  epsilon_dist_m: 0.05  # keep this" in changed
    assert "solver:\n  max_iterations: 1000" in changed
    assert changed.count("scene:") == 1
```

Test nested, inline-empty-map and absent paths.

- [ ] **Step 2: Add failing persistence tests**

```python
def test_write_refuses_while_commanding(tmp_path):
    path = write_fixture(tmp_path, scene="{}")
    before = path.read_bytes()
    source = scene_config.read_scene(path)["source_fnv1a64"]
    ok, reason = scene_config.write_scene(
        {"torso": valid_cylinder()}, source,
        path=path, commanding=lambda: True)
    assert not ok
    assert "controller is commanding" in reason
    assert path.read_bytes() == before
```

Add stale digest, invalid dimension, malformed disk scene, add/rename/delete, shape change, stable order, byte preservation and one-time `.panel.bak` tests.

- [ ] **Step 3: Run and confirm missing interfaces**

```bash
python3 -m pytest Christian_control/panel/tests/test_yaml_text.py \
  Christian_control/panel/tests/test_scene_config.py -q
```

- [ ] **Step 4: Implement read/validate/render/atomic write**

`read_scene` returns path, scene mapping, FNV-1a 64 token and error. Use the same FNV constants as C++:

```python
def _fnv1a64(data: bytes) -> int:
    value = 1469598103934665603
    for byte in data:
        value ^= byte
        value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return value
```

Validate schema/types/finite positive dimensions only—no collision. Render deterministic two-space YAML. Replace only `obstacles.scene` in a sibling temporary file; reparse the candidate and confirm all existing scalar knobs remain present through `planner_config.read_planner_knobs`; flush, `os.fsync` and `os.replace`. Return reread disk truth.

- [ ] **Step 5: Add isolated routes**

GET adds `save_allowed` and reason from `session.status()`. POST calls:

```python
ok, value = scene_config.write_scene(
    req.get("scene") or {},
    req.get("source_fnv1a64"),
    commanding=lambda: bool(session.status().get("commanding")))
```

Return 409 on rejection. Neither route calls solve, build, session start or subprocess.

- [ ] **Step 6: Test route isolation**

Monkeypatch `plan.solve`, `build.start_build` and `session.start` to raise if invoked, then exercise scene POST through a local handler test. Assert only `scene_config.write_scene` is reached.

- [ ] **Step 7: Run and commit**

```bash
python3 -m pytest Christian_control/panel/tests -q
git diff --check
git add Christian_control/panel/scene_config.py \
  Christian_control/panel/yaml_text.py \
  Christian_control/panel/planner_config.py \
  Christian_control/panel/server.py \
  Christian_control/panel/tests/test_yaml_text.py \
  Christian_control/panel/tests/test_scene_config.py
git commit -m "panel: edit the planner-owned obstacle scene"
```

---

### Task 5: Canonical Mount Placement and Multi-Primitive Rendering

**Files:**
- Create: `Christian_control/panel/tests/test_dh.py`
- Create: `Christian_control/panel/static/scene_geometry.mjs`
- Create: `Christian_control/panel/static/scene_geometry.test.mjs`
- Modify: `Christian_control/panel/paths.py`
- Modify: `Christian_control/panel/dh.py`
- Modify: `Christian_control/panel/static/scene.js`
- Modify: `Christian_control/panel/static/scene.test.html`

**Interfaces:**
- `/api/dh` adds `mount_from_base: {xyz:[...], rpy:[...]}`.
- `scene_geometry.mjs` exports `cylinderWireframe`, `boxWireframe` and `applyMountEdit`.
- scene API adds `setMountFromBase(arm, transform)`, `setObstacles(objects)` and `setSelectedObstacle(id)`.

- [ ] **Step 1: Add failing mount-source tests**

Parse a temporary `dual_arm_mounting.yaml` and assert:

```python
assert dh.read_mounting(path) == {
    "right": {"xyz": [0.0, -0.0375, 0.0], "rpy": [1.2085, 0.0, 0.0]},
    "left":  {"xyz": [0.0,  0.0375, 0.0], "rpy": [-1.2085, 0.0, 0.0]},
}
```

Reject missing/non-finite/negative separation and unsupported mount origin. `dh.read("right")` must include the served transform.

- [ ] **Step 2: Add failing pure display tests**

```javascript
test('cylinder uses full height around centre', () => {
  const wire = cylinderWireframe({
    id: 'torso', shape: 'cylinder',
    center_mount_m: [1, 2, 3], radius_m: 0.2, height_m: 0.6,
  }, 8);
  assert.equal(Math.min(...wire.points.map((p) => p[2])), 2.7);
  assert.equal(Math.max(...wire.points.map((p) => p[2])), 3.3);
  assert.deepEqual(wire.handles.radius, [1.2, 2, 3]);
  assert.deepEqual(wire.handles.height, [1, 2, 3.3]);
});
```

Test box corners, multiple objects, immutable inputs, centre edits, radius/height/extent handles, and absence of epsilon/collision fields.

- [ ] **Step 3: Run and confirm missing modules**

```bash
python3 -m pytest Christian_control/panel/tests/test_dh.py -q
node --test Christian_control/panel/static/scene_geometry.test.mjs
```

- [ ] **Step 4: Serve mounting data and remove production JS constants**

Add `paths.DUAL_ARM_MOUNTING`. Implement a narrow three-key parser. Keep generated DH behavior and add the selected arm's mount transform. Remove `DEFAULT_MOUNT_FROM_BASE` as production data; standalone page fixtures must be explicitly synthetic.

- [ ] **Step 5: Render collections**

Replace one `state.obstacle` with `obstacles: []` and `selectedObstacleId`. `setObstacles` normalizes only display values and does no SDF/grid work. Iterate enabled boxes/cylinders; selected disabled objects may appear ghosted only in editor mode. Update the eye page to draw multiple primitives and label them “configured primitive; planner evaluates clearance.”

- [ ] **Step 6: Run automated and visual checks**

```bash
python3 -m pytest Christian_control/panel/tests/test_dh.py -q
node --test Christian_control/panel/static/scene_geometry.test.mjs
python3 -m http.server 8001 -d Christian_control/panel/static
```

Use the in-app Browser on `http://127.0.0.1:8001/scene.test.html`. Check centred caps, coexistence, orbit and zoom. Stop the server.

- [ ] **Step 7: Commit**

```bash
git add Christian_control/panel/paths.py \
  Christian_control/panel/dh.py \
  Christian_control/panel/tests/test_dh.py \
  Christian_control/panel/static/scene_geometry.mjs \
  Christian_control/panel/static/scene_geometry.test.mjs \
  Christian_control/panel/static/scene.js \
  Christian_control/panel/static/scene.test.html
git commit -m "panel: render planner scene in the mount frame"
```

---

### Task 6: Saved/Draft Editor and Direct Manipulation

**Files:**
- Create: `Christian_control/panel/tests/test_static.py`
- Modify: `Christian_control/panel/static/index.html`
- Modify: `Christian_control/panel/static/panel.css`
- Modify: `Christian_control/panel/static/panel.js`
- Modify: `Christian_control/panel/static/scene.js`
- Modify: `Christian_control/panel/static/scene_geometry.mjs`
- Modify: `Christian_control/panel/static/scene_geometry.test.mjs`

**Interfaces:**

```javascript
sceneConfig: {
  saved: {},
  draft: {},
  sourceFnv1a64: null,
  selectedId: null,
  dirty: false,
  saveAllowed: false,
  saveBlockedReason: null,
}
```

`createScene(canvas, {onObstacleEdit})` emits `{id, obstacle, handle}` and makes no request.

- [ ] **Step 1: Extend handle tests first**

Test XY centre, Z centre, radius, full height and each box half extent. Assert only the intended field changes, input is immutable, and display floor is positive. Callback payload contains no URL/method/solve/command.

- [ ] **Step 2: Add static wiring tests**

Assert HTML contains edit/select/add/rename/delete, shape, centre, dimensions, reset/save and state controls. Extract `saveScene()` and assert it contains `/api/scene` but no `/api/plan/solve`, `/api/session/start`, `/api/build` or `/api/config/set`.

- [ ] **Step 3: Run and confirm editor absence**

```bash
node --test Christian_control/panel/static/scene_geometry.test.mjs
python3 -m pytest Christian_control/panel/tests/test_static.py -q
```

- [ ] **Step 4: Add one editor beside the existing canvas**

Keep one canvas/scene instance. An `EDIT SCENE` toggle reveals the inspector. Field or handle changes update one `replaceDraftObstacle(id, value)` function, redraw immediately, and display exactly:

```text
SAVED — planner disk truth
UNSAVED DRAFT — not used by planner
SAVE BLOCKED — controller is commanding
```

Add/rename/delete/shape changes affect draft only. Newly created drafts may use visibly unsaved 0.1 m display values; no default is written automatically.

- [ ] **Step 5: Preserve camera semantics while adding handles**

Hit-test selected handles before orbit. Empty canvas retains orbit; wheel retains zoom. XY handle moves in the current-Z mount plane, Z handle follows mount Z, and dimension handles follow their mount axes. `scene.js` and `scene_geometry.mjs` never call `fetch`.

- [ ] **Step 6: Implement reset and guarded save**

Reset deep-copies saved to draft. Save posts only:

```javascript
await postJSON('/api/scene', {
  scene: state.sceneConfig.draft,
  source_fnv1a64: state.sceneConfig.sourceFnv1a64,
});
```

Failure retains the draft/reason. Success replaces saved/draft with reread disk truth. Do not call goal refresh, solve, session start or build. Disable save when blocked but rely on server authority.

- [ ] **Step 7: Run automated tests**

```bash
node --test Christian_control/panel/static/scene_geometry.test.mjs
python3 -m pytest Christian_control/panel/tests -q
git diff --check
```

- [ ] **Step 8: Run hardware-free browser QA**

Start only:

```bash
python3 Christian_control/panel/control_panel.py --port 8765
```

With the in-app Browser, verify load, add torso draft, numeric X/Y/Z/radius/height, every handle, orbit/zoom, unsaved label, reset, one save, invalid fields, and network requests. Never press `START SESSION`. Restore checked-in empty scene and prove no test dimensions remain in the diff.

- [ ] **Step 9: Commit**

```bash
git add Christian_control/panel/static/index.html \
  Christian_control/panel/static/panel.css \
  Christian_control/panel/static/panel.js \
  Christian_control/panel/static/scene.js \
  Christian_control/panel/static/scene_geometry.mjs \
  Christian_control/panel/static/scene_geometry.test.mjs \
  Christian_control/panel/tests/test_static.py
git commit -m "panel: edit scene geometry visually and numerically"
```

---

### Task 7: Integrated Evidence, Documentation and Reviews

**Files:**
- Modify: `Christian_control/README.md`
- Modify: `docs/architecture.md`
- Verify: every Task 1–6 file.

**Interfaces:**
- Produces: hardware-free evidence of one YAML feeding panel and planner.

- [ ] **Step 1: Document verified flow and limitations**

Document:

```text
planner.yaml obstacles.scene
  -> PlannerConfig.scene
  -> MakeMountSdf
  -> GPMP2 factors
  -> traced-path modelled-clearance validation

same YAML
  -> GET /api/scene
  -> browser saved/draft
  -> mount renderer
  -> guarded atomic POST /api/scene
```

State point-plan final-sweep gap, no panel epsilon inflation, and planned versus executed/physical limitations.

- [ ] **Step 2: Build and test in a fresh temporary directory**

```bash
scene_build=$(mktemp -d /tmp/humansl-scene-build.XXXXXX)
cmake -S Christian_control/planning -B "$scene_build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$scene_build" -j2
ctest --test-dir "$scene_build" --output-on-failure
```

Do not delete or overwrite existing builds. Record exact counts.

- [ ] **Step 3: Run all panel/pure renderer tests**

```bash
python3 -m pytest Christian_control/panel/tests -q
node --test Christian_control/panel/static/scene_geometry.test.mjs
```

- [ ] **Step 4: Prove scene reaches planner diagnostics**

Use temporary copies of planner/goal config containing one in-grid `test_cylinder`. Run only the fresh `planner_bridge` with explicit start joints, fixed identity `world_T_mount`, nonzero Vicon sequence and trajectory ID. Assert diagnostics name `test_cylinder(cylinder)` and no legacy “operator box.” Never invoke `run_session.sh` or `controller`.

```bash
scene_tmp=$(mktemp -d /tmp/humansl-scene-config.XXXXXX)
cp Christian_control/planning/config/planner.yaml "$scene_tmp/planner.yaml"
cp Christian_control/planning/config/goal.yaml "$scene_tmp/goal.yaml"
# Use apply_patch, not a stream rewrite, to add one enabled in-grid test
# cylinder to the temporary planner copy before running this command.
LD_LIBRARY_PATH="$PWD/third_party/lib" \
  "$scene_build/planner_bridge" \
  --arm right \
  --goal-file "$scene_tmp/goal.yaml" \
  --planner-config "$scene_tmp/planner.yaml" \
  --joint-limits "$PWD/Christian_control/planning/config/joint_limits.yaml" \
  --dh "$scene_build/config/dh_params_tool.yaml" \
  --start-deg 0 0 0 0 0 0 0 \
  --world-mount-pose-m-quat 0 0 0 0 0 0 1 \
  --vicon-sequence 1 \
  --trajectory-id 1 \
  >"$scene_tmp/trajectory.txt" 2>"$scene_tmp/diagnostics.txt"
rg -n "test_cylinder\(cylinder\)" "$scene_tmp/diagnostics.txt"
! rg -n "operator box" "$scene_tmp/diagnostics.txt"
```

If the point solve fails for reachability, the SDF description must still be
present in diagnostics before accepting this check; otherwise use the
hardware-free `test_mount_sdf` integration fixture rather than changing the
goal to an unverified physical target.

- [ ] **Step 5: Prove save isolation and unchanged bytes on rejection**

Trace the browser edit/save: only scene GET/POST. Exercise `commanding=true` through the injected test seam, not a live process, and compare `planner.yaml` bytes before/after rejection.

- [ ] **Step 6: Run ownership searches**

```bash
rg -n "DeclaredBox|--box|box-on|request\\.obstacle" \
  Christian_control/planning Christian_control/panel \
  --glob '!**/build/**' --glob '!**/*.bak'
rg -n "hingeLoss|SignedDistance|epsilon_dist|collision_sigma" \
  Christian_control/panel --glob '!**/__pycache__/**'
rg -n "fetch\\(|postJSON\\(" \
  Christian_control/panel/static/scene.js \
  Christian_control/panel/static/scene_geometry.mjs
```

Expected: no active legacy source; panel only labels existing tuning; renderer makes no request.

- [ ] **Step 7: Apply full Level 2 safety/readability review**

Read `.agents/skills/kinova-safe-cpp/references/safety-review.md` and mark applicability. Confirm no forbidden dependency entered the core, both arms share one scene, no safety/stop thresholds changed, save cannot start motion, no physical torso value was invented, no diagnostic claims safety, and the two main paths remain readable.

- [ ] **Step 8: Request independent code/evidence review**

Use `superpowers:requesting-code-review`. Reviewer compares implementation to spec/packet, inspects the actual diff and asks whether wrong cylinder/frame physics could pass. Resolve every finding.

- [ ] **Step 9: Verify immediately before completion**

Use `superpowers:verification-before-completion` and rerun:

```bash
ctest --test-dir "$scene_build" --output-on-failure
python3 -m pytest Christian_control/panel/tests -q
node --test Christian_control/panel/static/scene_geometry.test.mjs
git diff --check
git status --short
```

Confirm feature commits never staged the user's pre-existing `CLAUDE.md` or raw prompt-log changes.

- [ ] **Step 10: Commit verified documentation**

```bash
git add Christian_control/README.md docs/architecture.md
git commit -m "docs: describe planner-owned obstacle scene"
```

- [ ] **Step 11: Report completion honestly**

Report files/classes/concepts added/removed, commands and counts, browser states checked, no robot executable run, hardware-free evidence label, point-plan limitation, planned/executed posture limitation, and unresolved physical torso dimensions/calibration.
