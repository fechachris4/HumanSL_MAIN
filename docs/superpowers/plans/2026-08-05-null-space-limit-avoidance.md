# Null-Space Joint-Limit Avoidance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace null-space midpoint centering with deadband joint-limit avoidance in the basic_control reactive law AND its paired Python simulation law, with regenerated cross-validation fixtures.

**Architecture:** The null-space channel (damped projector, `ReactiveSolution` decomposition, telemetry) stays exactly as-is; only the *objective* changes: zero everywhere inside the working range, a linear inward push once a bounded joint enters the activation zone `[limit − zone, limit]`. Spec: `docs/superpowers/specs/2026-08-05-null-space-limit-avoidance-design.md`.

**Tech Stack:** C++17/Eigen (header-only law), Python/NumPy (msc_project sim law), CMake/CTest, fixture generation via `~/msc_project/.venv/bin/python`.

## Global Constraints

- NEVER run `controller` or any Kortex-linked binary. Building is fine; running is never a test step (CLAUDE.md hardware rule).
- Never append a Co-Authored-By trailer to commits (user preference).
- Preserve the operator's uncommitted Config.h experiments: `kTakeoverHoldS = 0.05` stays untouched; `kNullGain = 23.0` / `kNullSpaceEnabled = false` are *deleted/superseded by this plan* (approved in the spec).
- Two repositories are touched: `/home/christian/Desktop/HumanSL_MAIN` (this repo) and `/home/christian/msc_project` (separate git repo — commit there separately).
- Build dir: `Christian_control/basic_control/cmake-build-debug` (ninja). Tests: `cd cmake-build-debug && ctest --output-on-failure`.
- All `git add` calls list explicit files — the working tree may carry unrelated operator edits at any time.
- Constants (exact, from the spec): `kLimitAvoidZoneDeg = 20.0`, `kLimitAvoidGain = 2.0` (1/s), zone anchored to `kJointSoftwareLimitDeg = {0, 126.9, 0, 145.0, 0, 118.0, 0}`.

---

### Task 0: Commit the pending telemetry work (pre-existing, already tested)

The working tree already contains tonight's finished, test-passing telemetry change (task/null decomposition, status line, log_format 8). It must land as its own commit so the law-swap diffs stay reviewable.

**Files:**
- Commit (already modified, do not edit): `Christian_control/basic_control/src/ReactiveLaw.h`, `src/State.h`, `src/Controller.cpp`, `src/Hardware.h`, `src/Hardware.cpp`, `src/Main.cpp`, `src/Runner.cpp`, `src/Safety.h`, `src/Safety.cpp`, `src/Config.h`, `scripts/runlog.py`, `tests/test_reactive_law.cpp`, `tests/test_supervisor.cpp`, `tests/test_runlog_compat.py`, and `Christian_control/docs/code-read/Hardware.md`, `Christian_control/docs/code-read/Main.md`.

**Interfaces:**
- Produces: a clean baseline where `SolveReactiveVelocityDetailed`, `ReactiveSolution`, `FormatStatusLine`, and log_format 8 exist on HEAD. Later tasks modify these files freely.

- [ ] **Step 1: Verify the baseline is green**

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control/cmake-build-debug && cd /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control/cmake-build-debug && ctest --output-on-failure`
Expected: 8/8 tests pass.

- [ ] **Step 2: Commit (explicit file list, both repos' paths relative to HumanSL_MAIN root)**

```bash
cd /home/christian/Desktop/HumanSL_MAIN
git add Christian_control/basic_control/src/ReactiveLaw.h \
        Christian_control/basic_control/src/State.h \
        Christian_control/basic_control/src/Controller.cpp \
        Christian_control/basic_control/src/Hardware.h \
        Christian_control/basic_control/src/Hardware.cpp \
        Christian_control/basic_control/src/Main.cpp \
        Christian_control/basic_control/src/Runner.cpp \
        Christian_control/basic_control/src/Safety.h \
        Christian_control/basic_control/src/Safety.cpp \
        Christian_control/basic_control/src/Config.h \
        Christian_control/basic_control/scripts/runlog.py \
        Christian_control/basic_control/tests/test_reactive_law.cpp \
        Christian_control/basic_control/tests/test_supervisor.cpp \
        Christian_control/basic_control/tests/test_runlog_compat.py \
        Christian_control/docs/code-read/Hardware.md \
        Christian_control/docs/code-read/Main.md
git commit -m "telemetry: reactive-law task/null decomposition, 1 Hz status line, log_format 8

Also carries two operator Config.h experiments made during the same
session: kTakeoverHoldS 0.5 -> 0.05 and kNullGain 23 (centering left
disabled). The centering constants are removed by the follow-up
limit-avoidance change."
```

---

### Task 1: Deadband limit-avoidance objective in the Python simulation law (msc_project repo)

**Files:**
- Modify: `/home/christian/msc_project/controller/reactive_controller.py` (equation 5, `solve_reactive_velocity` signature, `JointCentering` → `JointLimitAvoidance`, `ReactiveController.compute`)
- Modify: `/home/christian/msc_project/controller/servo.py` (type references, lines ~16/111/115)
- Modify: `/home/christian/msc_project/sim/world.py` (construction, lines ~14/124-129)
- Modify: `/home/christian/msc_project/runtime_config.py` (`ReactivePoseConfig` + `_REACTIVE_KEYS` + parser)
- Modify: `/home/christian/msc_project/config/control.toml` (add `limit_avoid_zone_rad`)
- Create: `/home/christian/msc_project/tests/test_limit_avoidance.py`

**Interfaces:**
- Consumes: existing `solve_reactive_velocity(jacobian_world, e_pos, e_rot, e_v, e_w, joint_position_rad, joint_midpoint_rad, null_gain_s_inv, config, damping=None)` and `JointCentering(midpoint_rad, enabled)`.
- Produces (Task 2's fixture generator imports these):
  - `solve_reactive_velocity(jacobian_world, e_pos, e_rot, e_v, e_w, joint_position_rad, joint_limit_rad, joint_zone_rad, null_gain_s_inv, config, damping=None)` — `joint_limit_rad` shape-(7,) with 0 = unbounded, `joint_zone_rad` scalar float.
  - `JointLimitAvoidance(limit_rad, zone_rad)` frozen dataclass (limit_rad shape-(7,), zone_rad float > 0).
  - `ReactivePoseConfig` keeps `null_gain_s_inv` (name unchanged in Python — it is the generic null-channel gain; the TOML/`K_NULL` legacy plumbing stays untouched) and gains new field `limit_avoid_zone_rad: float`.

- [ ] **Step 1: Write the failing test**

Create `/home/christian/msc_project/tests/test_limit_avoidance.py`:

```python
"""Deadband joint-limit avoidance: zero in the working range, linear
inward push inside the activation zone [limit - zone, limit]."""

import numpy as np
import pytest

from controller.reactive_controller import (
    JointLimitAvoidance,
    solve_reactive_velocity,
)
from runtime_config import ReactivePoseConfig


def make_config(**overrides):
    values = dict(
        kp_position_s_inv=1.0,
        kp_rotation_s_inv=1.0,
        kd_position=0.3,
        kd_rotation=0.3,
        null_gain_s_inv=2.0,
        limit_avoid_zone_rad=np.deg2rad(20.0),
        dls_damping=0.1,
        position_enabled=True,
        orientation_enabled=True,
        velocity_enabled=False,
    )
    values.update(overrides)
    return ReactivePoseConfig(**values)


LIMIT = np.deg2rad(np.array([0.0, 126.9, 0.0, 145.0, 0.0, 118.0, 0.0]))
ZONE = np.deg2rad(20.0)
ZERO3 = np.zeros(3)


def full_rank_jacobian():
    rows, cols = np.meshgrid(np.arange(6), np.arange(7), indexing="ij")
    return np.sin(1.0 + 3.0 * rows + 7.0 * cols) + 2.0 * (rows == cols)


def null_part(q):
    solve = solve_reactive_velocity(
        full_rank_jacobian(), ZERO3, ZERO3, ZERO3, ZERO3,
        q, LIMIT, ZONE, 2.0, make_config(),
    )
    return solve.qdot_null_objective, solve.qdot_null_projected


def test_objective_zero_everywhere_inside_zone():
    q = np.deg2rad(np.array([170.0, 60.0, -350.0, 110.0, 40.0, -90.0, 720.0]))
    objective, projected = null_part(q)
    assert np.all(objective == 0.0)
    assert np.all(projected == 0.0)


def test_push_is_inward_on_both_sides():
    q = np.zeros(7)
    q[3] = np.deg2rad(135.0)   # j4, 10 deg into its zone (entry 125)
    objective, _ = null_part(q)
    assert objective[3] == pytest.approx(-2.0 * np.deg2rad(10.0))
    q[3] = np.deg2rad(-135.0)
    objective, _ = null_part(q)
    assert objective[3] == pytest.approx(2.0 * np.deg2rad(10.0))


def test_magnitude_at_limit_is_gain_times_zone():
    q = np.zeros(7)
    q[5] = np.deg2rad(118.0)   # j6 exactly at its software limit
    objective, _ = null_part(q)
    assert objective[5] == pytest.approx(-2.0 * ZONE)


def test_unbounded_joints_never_push():
    q = np.zeros(7)
    q[0] = np.deg2rad(359.0)   # continuous joint, any position
    objective, _ = null_part(q)
    assert objective[0] == 0.0


def test_wrap_uses_nearest_turn():
    q = np.zeros(7)
    q[5] = np.deg2rad(250.0)   # Kortex-style reading; signed = -110, in zone
    objective, _ = null_part(q)
    assert objective[5] == pytest.approx(2.0 * np.deg2rad(12.0))


def test_dataclass_validation():
    with pytest.raises(ValueError):
        JointLimitAvoidance(limit_rad=np.zeros(3), zone_rad=ZONE)
    with pytest.raises(ValueError):
        JointLimitAvoidance(limit_rad=np.zeros(7), zone_rad=0.0)
```

- [ ] **Step 2: Run it to make sure it fails**

Run: `cd /home/christian/msc_project && .venv/bin/python -m pytest tests/test_limit_avoidance.py -x -q`
Expected: FAIL — `ImportError: cannot import name 'JointLimitAvoidance'`.

- [ ] **Step 3: Implement the law change**

In `controller/reactive_controller.py`:

3a. Replace equation 5 inside `solve_reactive_velocity` (and its two parameters). New signature and equations 5–6:

```python
def solve_reactive_velocity(
    jacobian_world,
    e_pos,
    e_rot,
    e_v,
    e_w,
    joint_position_rad,
    joint_limit_rad,
    joint_zone_rad,
    null_gain_s_inv,
    config,
    damping=None,
):
    """Equations 3-6: PD + DLS + null-space requested joint velocity."""
    damping = config.dls_damping if damping is None else damping

    # Equation 3: desired world-frame task twist.
    p_twist, d_twist = task_twist_terms(
        e_pos, e_rot, e_v, e_w, config)
    task_twist = p_twist + d_twist

    # Equation 4: damped least-squares inverse kinematics.
    qdot_task = jacobian_world.T @ np.linalg.solve(
        jacobian_world @ jacobian_world.T
        + damping**2 * np.eye(6),
        task_twist,
    )

    # Equation 5: deadband joint-limit avoidance. Zero across the whole
    # working range; a linear inward push once a bounded joint enters the
    # activation zone [limit - zone, limit]. Unbounded joints have limit 0.
    limit = np.asarray(joint_limit_rad, dtype=float)
    signed = np.remainder(
        np.asarray(joint_position_rad, dtype=float) + np.pi, 2.0 * np.pi
    ) - np.pi
    excess = np.abs(signed) - (limit - float(joint_zone_rad))
    qdot_null_objective = np.where(
        (limit > 0.0) & (excess > 0.0),
        -np.asarray(null_gain_s_inv) * excess * np.sign(signed),
        0.0,
    )

    # Equation 6: project the push into the Jacobian null space.
    qdot_null_projected = (
        np.eye(7) - np.linalg.pinv(jacobian_world) @ jacobian_world
    ) @ qdot_null_objective
    return ReactiveSolve(
        p_twist=p_twist,
        d_twist=d_twist,
        task_twist=task_twist,
        qdot_task=qdot_task,
        qdot_null_objective=qdot_null_objective,
        qdot_null_projected=qdot_null_projected,
        qdot_raw=qdot_task + qdot_null_projected,
    )
```

3b. Replace the `JointCentering` dataclass with:

```python
@dataclass(frozen=True, slots=True)
class JointLimitAvoidance:
    limit_rad: np.ndarray  # shape (7,), 0 = unbounded joint
    zone_rad: float

    def __post_init__(self):
        limit = np.asarray(self.limit_rad, dtype=float)
        if limit.shape != (7,) or not np.all(np.isfinite(limit)) or np.any(limit < 0.0):
            raise ValueError("limit_rad must be a finite non-negative shape-(7,) array")
        zone = float(self.zone_rad)
        if not np.isfinite(zone) or zone <= 0.0:
            raise ValueError("zone_rad must be a finite positive float")
        object.__setattr__(self, "limit_rad", _read_only(limit))
        object.__setattr__(self, "zone_rad", zone)
```

3c. Update `ReactiveController` (`__init__` parameter `centering` → `avoidance`, `isinstance` check against `JointLimitAvoidance`, and `compute`):

```python
        solve = solve_reactive_velocity(
            state.jacobian_world,
            e_pos,
            e_rot,
            e_v,
            e_w,
            state.joints.position_rad,
            self._avoidance.limit_rad,
            self._avoidance.zone_rad,
            self._config.null_gain_s_inv,
            self._config,
        )
```

3d. Update the module docstring list item 5: "null-space joint centering" → "null-space joint-limit avoidance (deadband)".

- [ ] **Step 4: Update the consumers**

4a. `controller/servo.py`: change the import and the two `JointCentering` type references (field `centering: JointCentering` → `avoidance: JointLimitAvoidance` and its isinstance check). Follow the compiler: grep `centering` in servo.py and rename each use to `avoidance`.

4b. `sim/world.py` lines ~124-129: replace midpoint construction with limits:

```python
        limit = np.zeros(7)
        limit[limited] = np.maximum(
            np.abs(lower[limited]), np.abs(upper[limited])
        )
        ...
            avoidance=JointLimitAvoidance(
                limit, CONFIG.controller.reactive_pose.limit_avoid_zone_rad
            ),
```

(where `lower`/`upper` are the same per-joint range arrays the midpoint code already reads — keep the surrounding code's actual variable names.)

4c. `runtime_config.py`: add `limit_avoid_zone_rad: float` to `ReactivePoseConfig` (after `null_gain_s_inv`); add `"limit_avoid_zone_rad"` to `_REACTIVE_KEYS`; add a parser line next to the `null_gain_s_inv` one:

```python
        limit_avoid_zone_rad=_finite_number(
            table["limit_avoid_zone_rad"],
            "controller.reactive_pose.limit_avoid_zone_rad",
            positive=True,
        ),
```

(match the existing `_finite_number` call style at line ~455 exactly.)

4d. `config/control.toml`: next to `null_gain_s_inv = 1.0` add `limit_avoid_zone_rad = 0.349` (20° in rad) and a one-line comment `# activation zone below each software joint limit, rad`.

- [ ] **Step 5: Run the new test until green**

Run: `cd /home/christian/msc_project && .venv/bin/python -m pytest tests/test_limit_avoidance.py -x -q`
Expected: all pass.

- [ ] **Step 6: Run the whole msc_project suite; regenerate the golden trace**

Run: `cd /home/christian/msc_project && .venv/bin/python -m pytest -x -q`
Expected: golden-trace tests FAIL (the law changed — that is the trace doing its job). Anything else failing means a broken rename: fix before proceeding.
Then regenerate and re-check:

```bash
.venv/bin/python -m tests.golden_trace --record
.venv/bin/python -m pytest -x -q
```

Expected: all pass.

- [ ] **Step 7: Commit (msc_project repo)**

```bash
cd /home/christian/msc_project
git add controller/reactive_controller.py controller/servo.py sim/world.py \
        runtime_config.py config/control.toml tests/test_limit_avoidance.py \
        tests/golden/
git commit -m "controller: replace null-space centering with deadband joint-limit avoidance"
```

---

### Task 2: C++ law swap + regenerated cross-validation fixtures (HumanSL_MAIN repo)

One atomic task: the C++ objective, the fixture generator, and the regenerated header must land together for the tests to be green.

**Files:**
- Modify: `Christian_control/basic_control/src/ReactiveLaw.h`
- Modify: `Christian_control/basic_control/tests/test_reactive_law.cpp`
- Modify: `Christian_control/basic_control/scripts/gen_reactive_fixtures.py`
- Regenerate: `Christian_control/basic_control/tests/reactive_fixtures.h`

**Interfaces:**
- Consumes: Task 1's Python `solve_reactive_velocity(..., joint_limit_rad, joint_zone_rad, null_gain_s_inv, config)`.
- Produces (Task 3 wires these):
  - `LimitAvoidanceVelocity(const Eigen::Matrix<double,6,7>& jacobian, const Eigen::Matrix<double,7,1>& q_rad, const Eigen::Matrix<double,7,1>& limit_rad, double zone_rad, const ReactivePoseGains& gains) -> Eigen::Matrix<double,7,1>`
  - `ReactivePoseGains::limit_avoid_gain_s_inv` (renamed from `null_gain_s_inv`)
  - `SolveReactiveVelocityDetailed(jacobian, e_pos, e_rot, e_v, e_w, q_rad, limit_rad, zone_rad, gains) -> ReactiveSolution` and `SolveReactiveVelocity(...)` with the same two replaced parameters (`midpoint_rad`, `centering_mask` are gone).

- [ ] **Step 1: Write the failing tests**

In `tests/test_reactive_law.cpp`: delete `TestNullSpace()` and its call in `main()`; replace with `TestLimitAvoidance()`; update `DefaultGains()` (`gains.null_gain_s_inv = 0.5;` → `gains.limit_avoid_gain_s_inv = 0.5;`).

```cpp
    void TestLimitAvoidance()
    {
        ReactivePoseGains gains = DefaultGains();
        gains.limit_avoid_gain_s_inv = 2.0;
        const double kDeg = M_PI / 180.0;
        Eigen::Matrix<double, 7, 1> limit;
        limit << 0, 126.9 * kDeg, 0, 145.0 * kDeg, 0, 118.0 * kDeg, 0;
        const double zone = 20.0 * kDeg;

        // Zero Jacobian + damping: projector = identity — isolates the
        // objective, exactly as the old centering test did.
        const Eigen::Matrix<double, 6, 7> no_task =
            Eigen::Matrix<double, 6, 7>::Zero();
        Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();

        // Everywhere inside the zone: EXACTLY zero, including wrapped and
        // multi-turn positions on unbounded joints.
        q << 170 * kDeg, 60 * kDeg, -350 * kDeg, 110 * kDeg, 40 * kDeg,
             -90 * kDeg, 720 * kDeg;
        auto qdot = LimitAvoidanceVelocity(no_task, q, limit, zone, gains);
        Check(qdot.norm() == 0.0,
              "objective is exactly zero everywhere inside the zones");

        // 10 deg into j4's zone (entry 125): push is inward, k * excess.
        q.setZero();
        q[3] = 135.0 * kDeg;
        qdot = LimitAvoidanceVelocity(no_task, q, limit, zone, gains);
        Check(std::abs(qdot[3] - (-2.0 * 10.0 * kDeg)) < 1e-12,
              "positive-side push is inward and linear in the excess");
        q[3] = -135.0 * kDeg;
        qdot = LimitAvoidanceVelocity(no_task, q, limit, zone, gains);
        Check(std::abs(qdot[3] - (2.0 * 10.0 * kDeg)) < 1e-12,
              "negative-side push is inward and linear in the excess");

        // At the limit itself the magnitude is k * zone.
        q.setZero();
        q[5] = 118.0 * kDeg;
        qdot = LimitAvoidanceVelocity(no_task, q, limit, zone, gains);
        Check(std::abs(qdot[5] - (-2.0 * zone)) < 1e-12,
              "push at the limit equals gain times zone width");

        // Kortex-style wrapped reading: 250 deg = signed -110, in j6's zone.
        q.setZero();
        q[5] = 250.0 * kDeg;
        qdot = LimitAvoidanceVelocity(no_task, q, limit, zone, gains);
        Check(std::abs(qdot[5] - (2.0 * 12.0 * kDeg)) < 1e-12,
              "objective wraps the position to the nearest turn");

        // Unbounded joints (limit 0) never push, at any position.
        q.setZero();
        q[0] = 359.0 * kDeg;
        q[2] = -2.5;
        qdot = LimitAvoidanceVelocity(no_task, q, limit, zone, gains);
        Check(qdot.norm() == 0.0, "unbounded joints never push");

        // With an undamped full-rank Jacobian the projected push stays in
        // the Jacobian null space: J * qdot_null = 0.
        gains.dls_lambda = 0.0;
        Eigen::Matrix<double, 6, 7> jacobian;
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 7; ++c)
                jacobian(r, c) =
                    std::sin(1.0 + (3.0 * r) + (7.0 * c)) + (r == c ? 2.0 : 0.0);
        q.setZero();
        q[3] = 140.0 * kDeg;
        qdot = LimitAvoidanceVelocity(jacobian, q, limit, zone, gains);
        Check(qdot.norm() > 0.0, "avoidance produces motion inside the zone");
        Check((jacobian * qdot).norm() < (1e-9 * qdot.norm()),
              "projected push lies in the Jacobian null space");

        // SolveReactiveVelocity only adds it when enabled.
        gains = DefaultGains();
        const Eigen::Vector3d zero3 = Eigen::Vector3d::Zero();
        q.setZero();
        q[3] = 140.0 * kDeg;
        gains.null_space_enabled = false;
        auto without = SolveReactiveVelocity(jacobian, zero3, zero3, zero3,
                                             zero3, q, limit, zone, gains);
        Check(without.norm() == 0.0, "null-space disabled -> no avoidance motion");
        gains.null_space_enabled = true;
        auto with = SolveReactiveVelocity(jacobian, zero3, zero3, zero3,
                                          zero3, q, limit, zone, gains);
        Check(with.norm() > 0.0, "null-space enabled -> avoidance motion");
    }
```

Also update `TestDetailedSolveDecomposition()` in place: replace its `midpoint`/`mask` locals with `limit` (the 7-vector above) and `zone`, put a bounded joint inside its zone (`q[3] = 140.0 * kDeg`), and change every call: `SolveReactiveVelocityDetailed(jacobian, e_pos, e_rot, e_v, e_w, q, limit, zone, gains)`, the `NullSpaceVelocity(...)` reference becomes `LimitAvoidanceVelocity(jacobian, q, limit, zone, gains)`, and `gains.null_gain_s_inv = 5.0` becomes `gains.limit_avoid_gain_s_inv = 5.0`. The assertions themselves (parts sum to total, task part is the DLS solution, leak twist is the Jacobian image, disabled → zeros) stay word-for-word.

Update the two remaining `null_gain_s_inv` references (`DefaultGains()` at line ~51 and the fixture mapping at line ~358) — the fixture mapping is fully rewritten in Step 5.

- [ ] **Step 2: Run to verify failure**

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control/cmake-build-debug --target test_reactive_law`
Expected: FAIL to compile — `'LimitAvoidanceVelocity' was not declared`, `'struct ReactivePoseGains' has no member named 'limit_avoid_gain_s_inv'`.

- [ ] **Step 3: Implement in ReactiveLaw.h**

Delete `NullSpaceVelocity` and replace with (same position in the file):

```cpp
// Equations 5-6: deadband joint-limit avoidance projected into the
// Jacobian null space. `limit_rad` holds each joint's software-limit
// magnitude (applied symmetrically, ±limit); 0 marks an unbounded joint.
// The objective is EXACTLY zero until a bounded joint's wrapped position
// enters the activation zone [limit − zone, limit], then pushes inward
// with gain · excess. Replaced wrap-to-midpoint centering on 2026-08-05:
// centering fought the task from everywhere and the damped projector
// leaked it into task space (218 mm stall equilibrium) — see
// docs/superpowers/specs/2026-08-05-null-space-limit-avoidance-design.md.
inline Eigen::Matrix<double, 7, 1>
LimitAvoidanceVelocity(const Eigen::Matrix<double, 6, 7>& jacobian,
                       const Eigen::Matrix<double, 7, 1>& q_rad,
                       const Eigen::Matrix<double, 7, 1>& limit_rad,
                       double zone_rad,
                       const ReactivePoseGains& gains)
{
    // Equation 5: the deadband objective (wrapped to (−π, π] because
    // Kortex reports positions in [0, 360)).
    Eigen::Matrix<double, 7, 1> objective;
    for (int i = 0; i < 7; ++i) {
        objective[i] = 0.0;
        if (limit_rad[i] <= 0.0)
            continue; // unbounded joint
        const double signed_rad = std::remainder(q_rad[i], 2.0 * M_PI);
        const double excess =
            std::abs(signed_rad) - (limit_rad[i] - zone_rad);
        if (excess > 0.0)
            objective[i] = -gains.limit_avoid_gain_s_inv * excess *
                           (signed_rad < 0.0 ? -1.0 : 1.0);
    }

    // Equation 6: N = I₇ − Jᵀ(JJᵀ + λ²I₆)⁻¹J, the damped projector.
    Eigen::Matrix<double, 6, 6> jjt = jacobian * jacobian.transpose();
    jjt.diagonal().array() += gains.dls_lambda * gains.dls_lambda;
    const Eigen::Matrix<double, 7, 7> projector =
        Eigen::Matrix<double, 7, 7>::Identity() -
        jacobian.transpose() * jjt.ldlt().solve(jacobian);
    return projector * objective;
}
```

In `ReactivePoseGains`: rename `null_gain_s_inv` → `limit_avoid_gain_s_inv` (comment: `// 1/s on the zone excess`). In `SolveReactiveVelocityDetailed` and `SolveReactiveVelocity`: replace the two parameters `const Eigen::Matrix<double,7,1>& midpoint_rad, const Eigen::Matrix<double,7,1>& centering_mask` with `const Eigen::Matrix<double,7,1>& limit_rad, double zone_rad`, and the objective call becomes `LimitAvoidanceVelocity(jacobian, q_rad, limit_rad, zone_rad, gains)`. Update the file-header equation list (line 5: "joint centering" → "joint-limit avoidance (deadband)").

NOTE: `src/Controller.cpp` still calls the old signature after this step — the `controller` target will not build until Task 3. That is expected; only `test_reactive_law` must build here.

- [ ] **Step 4: Run the law tests (fixture case still pending)**

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control/cmake-build-debug --target test_reactive_law && /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control/cmake-build-debug/test_reactive_law`
Expected: compile errors ONLY in `TestAgainstSimulationFixtures` (old fixture fields) — proceed to Step 5, which replaces both the generator and the mapping.

- [ ] **Step 5: Update the fixture generator and the mapping**

5a. `scripts/gen_reactive_fixtures.py`:
- `MSC_PROJECT = Path.home() / "msc_project"` (fix the stale path).
- Follow the existing emission code: replace the per-case `MID` (midpoints) and `null_gain * MASK` arguments with the new law's `LIMIT` and `ZONE` (define once: `LIMIT = np.deg2rad(np.array([0.0, 126.9, 0.0, 145.0, 0.0, 118.0, 0.0]))`, `ZONE = np.deg2rad(20.0)`), pass `null_gain` as the scalar gain, and emit two extra struct fields per case: `double limit_rad[7]; double zone_rad;` with the C++ field `limit_avoid_gain_s_inv` replacing `null_gain_s_inv`.
- Replace the null-space case list entries: keep every existing task-only case (gain 0.0, tolerance 1e-12) unchanged; replace the `null_space_tiny_damping` case with two new ones, generated with `dls_damping=1e-8`:
  - `avoidance_active`: `q[3] = np.deg2rad(140.0)` (inside j4's zone), gain 2.0, tolerance 1e-6;
  - `avoidance_inactive`: all bounded joints ≥ 10° outside their zones, gain 2.0, tolerance 1e-12 — the emitted expectation must equal the gain-0 solve exactly (objective is exactly zero, not merely small).

5b. Regenerate: `cd /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control && /home/christian/msc_project/.venv/bin/python scripts/gen_reactive_fixtures.py`

5c. Rewrite the body of `TestAgainstSimulationFixtures` mapping loop to the new fields (keep its structure and Check message style):

```cpp
        for (const auto& c : fixtures::kSolveCases) {
            Eigen::Matrix<double, 6, 7> jacobian;
            for (int r = 0; r < 6; ++r)
                for (int col = 0; col < 7; ++col)
                    jacobian(r, col) = c.jacobian[r * 7 + col];
            Eigen::Matrix<double, 7, 1> q, limit;
            for (int i = 0; i < 7; ++i) {
                q[i] = c.q[i];
                limit[i] = c.limit_rad[i];
            }
            ReactivePoseGains gains;
            gains.kp_position_s_inv = c.kp_position_s_inv;
            gains.kp_rotation_s_inv = c.kp_rotation_s_inv;
            gains.kd_position = c.kd_position;
            gains.kd_rotation = c.kd_rotation;
            gains.limit_avoid_gain_s_inv = c.limit_avoid_gain_s_inv;
            gains.dls_lambda = c.dls_lambda;
            gains.position_enabled = c.position_enabled;
            gains.orientation_enabled = c.orientation_enabled;
            gains.velocity_enabled = c.velocity_enabled;
            gains.null_space_enabled = c.null_space_enabled;
            const auto qdot = SolveReactiveVelocity(
                jacobian, Eigen::Vector3d(c.e_pos), Eigen::Vector3d(c.e_rot),
                Eigen::Vector3d(c.e_v), Eigen::Vector3d(c.e_w), q, limit,
                c.zone_rad, gains);
            for (int i = 0; i < 7; ++i)
                Check(std::abs(qdot[i] - c.expected[i]) < c.tolerance,
                      std::string("fixture ") + c.name + " joint " +
                          std::to_string(i));
        }
```

(Adapt the Eigen construction lines to match how the current mapping builds vectors from the case arrays — keep whatever compiles against the regenerated header.)

- [ ] **Step 6: Run to green**

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control/cmake-build-debug --target test_reactive_law && /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control/cmake-build-debug/test_reactive_law`
Expected: `all reactive-law tests passed`.

- [ ] **Step 7: Commit**

```bash
cd /home/christian/Desktop/HumanSL_MAIN
git add Christian_control/basic_control/src/ReactiveLaw.h \
        Christian_control/basic_control/tests/test_reactive_law.cpp \
        Christian_control/basic_control/scripts/gen_reactive_fixtures.py \
        Christian_control/basic_control/tests/reactive_fixtures.h
git commit -m "controller: deadband joint-limit avoidance replaces centering in the reactive law"
```

---

### Task 3: Wire the new objective through Config, Controller, and the CSV preamble

**Files:**
- Modify: `Christian_control/basic_control/src/Config.h`
- Modify: `Christian_control/basic_control/src/Controller.h` (members `null_midpoint_rad_`, `null_centering_mask_`)
- Modify: `Christian_control/basic_control/src/Controller.cpp`
- Modify: `Christian_control/basic_control/src/Main.cpp` (preamble lines)
- Modify: `Christian_control/basic_control/tests/test_control_logic.cpp` (`TestJointPositionConfiguration`)

**Interfaces:**
- Consumes: Task 2's `LimitAvoidanceVelocity` / renamed gains / new solve signatures.
- Produces: `config::kLimitAvoidZoneDeg`, `config::kLimitAvoidGain`; `kNullGain`, `kNullMidpointDeg`, `kNullCenteringMask` no longer exist anywhere.

- [ ] **Step 1: Write the failing config test**

In `tests/test_control_logic.cpp`, inside `TestJointPositionConfiguration()`, replace the two centering checks (`kNullMidpointDeg ...` and `kNullCenteringMask ...`) with:

```cpp
        Check(config::kLimitAvoidZoneDeg == 20.0,
              "avoidance zone is 20 deg below each software limit");
        Check(config::kLimitAvoidGain == 2.0,
              "avoidance gain is the staged 2.0 1/s");
        Check(config::kNullSpaceEnabled,
              "the null-space channel ships enabled with limit avoidance");
        Check(config::kLimitAvoidZoneDeg <
                  config::kJointSoftwareLimitDeg[5],
              "zone width stays inside the narrowest bounded joint's limit");
```

- [ ] **Step 2: Run to verify failure**

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control/cmake-build-debug --target test_control_logic`
Expected: FAIL to compile — `kLimitAvoidZoneDeg` not a member of `config`.

- [ ] **Step 3: Config.h**

Delete `kNullGain` (line ~126), the whole `kNullMidpointDeg` initializer block (lines ~146-154), and `kNullCenteringMask` (line ~155). Set `kNullSpaceEnabled = true` and update the staging comment (line ~121-122) to "orientation, Kd term, and null-space limit avoidance ON". Immediately after the `kJointSoftwareLimitDeg` block add:

```cpp
    // Deadband null-space limit avoidance (ReactiveLaw.h). The objective is
    // exactly zero until a bounded joint's wrapped position comes within
    // kLimitAvoidZoneDeg of its software limit above, then pushes inward at
    // kLimitAvoidGain (1/s) times the excess — at the limit that is
    // gain × zone ≈ 40 deg/s before projection, bounded by the per-joint
    // clip. Replaced midpoint centering 2026-08-05:
    // docs/superpowers/specs/2026-08-05-null-space-limit-avoidance-design.md.
    inline constexpr double kLimitAvoidZoneDeg = 20.0;
    inline constexpr double kLimitAvoidGain = 2.0;
```

- [ ] **Step 4: Controller.h / Controller.cpp**

`Controller.h`: rename the two member declarations `null_midpoint_rad_` / `null_centering_mask_` to `limit_rad_` (same Eigen 7-vector type) and delete the second; add `double zone_rad_ = 0.0;`.

`Controller.cpp`:
- `ConfiguredGains()`: `gains.null_gain_s_inv = config::kNullGain;` → `gains.limit_avoid_gain_s_inv = config::kLimitAvoidGain;`
- Constructor loop becomes:

```cpp
    for (int i = 0; i < 7; ++i)
        limit_rad_[i] = config::kJointSoftwareLimitDeg[i] * kDegToRad;
    zone_rad_ = config::kLimitAvoidZoneDeg * kDegToRad;
```

- `DesiredVelocity()`: `ramped_gains.null_gain_s_inv *=` → `ramped_gains.limit_avoid_gain_s_inv *=`; the solve call becomes `SolveReactiveVelocityDetailed(ee.jacobian, e_pos, e_rot, e_twist.linear_m_s, e_twist.angular_rad_s, state.q_rad, limit_rad_, zone_rad_, ramped_gains);`

- [ ] **Step 5: Main.cpp preamble**

Replace `line("null_gain", FormatDouble(config::kNullGain));` with:

```cpp
    line("limit_avoid_zone_deg", FormatDouble(config::kLimitAvoidZoneDeg));
    line("limit_avoid_gain", FormatDouble(config::kLimitAvoidGain));
```

(`null_space_enabled` line stays; no `log_format` bump — only `#` preamble keys change.)

- [ ] **Step 6: Full build and suite**

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control/cmake-build-debug && cd /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control/cmake-build-debug && ctest --output-on-failure`
Expected: every target builds (controller included — build only, never run) and 8/8 tests pass.
Then: `grep -rn 'kNullGain\|kNullMidpointDeg\|kNullCenteringMask\|null_gain_s_inv' /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control/src /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control/tests /home/christian/Desktop/HumanSL_MAIN/Christian_control/basic_control/scripts`
Expected: no matches.

- [ ] **Step 7: Commit**

```bash
cd /home/christian/Desktop/HumanSL_MAIN
git add Christian_control/basic_control/src/Config.h \
        Christian_control/basic_control/src/Controller.h \
        Christian_control/basic_control/src/Controller.cpp \
        Christian_control/basic_control/src/Main.cpp \
        Christian_control/basic_control/tests/test_control_logic.cpp
git commit -m "config: wire deadband limit avoidance; remove centering constants; null space on"
```

---

### Task 4: Documentation sweep

**Files:**
- Modify: whichever `Christian_control/docs/code-read/*.md` files mention centering/midpoints (found by grep in Step 1 — expected: the ReactiveLaw/Controller/Config chapters).

**Interfaces:**
- Consumes: the final state of Tasks 2-3.
- Produces: code-read docs consistent with source; no stale `kNullGain`/centering references anywhere in docs.

- [ ] **Step 1: Find every stale mention**

Run: `grep -rn -i 'centering\|midpoint\|kNullGain' /home/christian/Desktop/HumanSL_MAIN/Christian_control/docs/code-read/`
Expected: a handful of hits in the ReactiveLaw/Controller/Config chapters.

- [ ] **Step 2: Update each hit**

For each hit, rewrite the sentence to describe the deadband limit avoidance (objective zero inside the zone `[limit − kLimitAvoidZoneDeg, limit]`, linear inward push, gain `kLimitAvoidGain`, anchor `kJointSoftwareLimitDeg`, spec reference `docs/superpowers/specs/2026-08-05-null-space-limit-avoidance-design.md`). Verify each rewritten claim against the current source before writing it (names, values, line behavior) — code-read docs must never drift ahead of or behind the code.

- [ ] **Step 3: Verify and commit**

Run the Step 1 grep again. Expected: no hits describing centering as current behavior (historical notes marked as removed are fine).

```bash
cd /home/christian/Desktop/HumanSL_MAIN
git add Christian_control/docs/code-read/
git commit -m "docs: code-read chapters follow the limit-avoidance law swap"
```

---

## Completion criteria (whole plan)

- Both repos committed as above; `ctest` 8/8 in basic_control; full msc_project pytest green.
- `grep -rn 'kNullGain\|NullSpaceVelocity\|kNullMidpointDeg\|kNullCenteringMask'` over `basic_control/{src,tests,scripts}` returns nothing.
- Hardware validation is NOT part of this plan: it requires Christian's explicit authorization, his presence, a clear workspace, and the e-stop — first run watches the status line for `null=0.0 leak=0.000` in the working range and a bounded push (no `joint_limit_warning` stop) when a joint enters its zone.
