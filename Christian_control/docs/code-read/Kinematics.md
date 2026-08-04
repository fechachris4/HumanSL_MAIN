# Kinematics.cpp / Kinematics.h — line-by-line read

*(Updated for commit f64325c0: the source files are unchanged by the
trajectory-playback removal; the Main/Controller call-site line numbers
below have moved, and Main's playback FK cross-check caller is gone.)*

Kinematics is the Pinocchio robot-model layer: given joint angles, where is
the tool and how does it move per unit joint velocity (the Jacobian). Two
halves:

1. **Generic free functions** (`forward_kinematics`,
   `position_and_jacobian`, `pose_and_jacobian`) — thin wrappers over
   Pinocchio for any frame of any model.
2. **`DualArmKinematics`** — the adapter that lets a 14-DoF two-arm URDF
   serve a 7-joint right-arm controller. This is what the run actually
   uses.

Execution order in a real run:

1. `Dynamics dynamics(GEN3_DUAL_URDF_PATH)` — Main.cpp:228 (Dynamics is
   TrajectoryExecution's URDF loader, not part of this file — it parses the
   URDF into `model_` and allocates the scratch `data_`).
2. `DualArmKinematics controlled_model(...)` — Main.cpp:229-232
   (constructor validates everything).
3. `KinematicsWorkspace workspace(model.dynamics())` — Main.cpp startup-pose
   measurement and Controller.cpp:33
   (preallocation).
4. `RightPoseAndJacobian(q, workspace)` — Main.cpp startup-pose measurement
   and Controller.cpp:47/83 —
   **every control cycle**.

The generic free functions are *not* called by the controller binary at
all: `forward_kinematics` is used by tests, `RightPositionAndJacobian` by
the probe_direction tool, and free `pose_and_jacobian` /
`position_and_jacobian` by nothing in the run. Details below.

---

## The types (Kinematics.h)

- `Pose` (h:32-35) — position (meters) + rotation matrix. A rotation
  matrix is a 3×3 orthonormal matrix whose columns are the frame's axes
  expressed in the reference frame.
- `KinematicsWorkspace` (h:47-54) — **why it exists**: Pinocchio's frame
  Jacobian for this model is 6×14 and Eigen would heap-allocate it on
  every query. The cyclic loop may not allocate, so the matrix is built
  once, outside the loop, and reused. The constructor calls `setZero()`
  because `getFrameJacobian` only writes the *nonzero* entries — reusing a
  dirty buffer would leave stale numbers in columns the frame does not
  depend on. **FLAG `Kinematics.h:47-54` | edit-hazard** — the
  `setZero()` looks removable ("it gets overwritten anyway") and is not;
  `UpdateFullKinematics` re-zeros defensively (cpp:249) for the same
  reason.
- `PositionJacobian` (h:60-64) — position + rotation + the 3×7
  *translational* Jacobian (rows x,y,z; columns joints 1-7).
- `PoseJacobian` (h:72-76) — the full 6×7 version, rows
  [linear; angular]. The header comments hammer one invariant: position,
  rotation and Jacobian rows must all be expressed in the *same declared
  frame* and describe the *same configuration*.

## The generic free functions (Kinematics.cpp:27-82)

### `forward_kinematics` (cpp:27-43)

- **Lines 30-32** — refuse a frame name the URDF does not define, with a
  message pointing at the link names. Checking first matters because
  `getFrameId` on a missing name returns a garbage index rather than
  failing.
- **Line 36** — `framesForwardKinematics`: one walk of the kinematic tree
  placing every joint for configuration `q_pin`, then updating the frames
  attached to them.
- **Lines 39-42** — `data_.oMf[frame_id]` ("origin-to-frame") is the
  frame's pose in the model root frame; return its translation and
  rotation.
- **FLAG `Kinematics.cpp:27-43` | unnecessary** — no caller in the
  controller binary (tests only: test_dual_arm_model.cpp). Harmless, but
  not part of the run.

### `position_and_jacobian` (cpp:45-64) and `pose_and_jacobian` (cpp:66-82)

Both use the same one-tree-walk pattern:
`computeJointJacobians` (which also computes joint placements) →
`updateFramePlacements` → `getFrameJacobian` into the workspace. Doing it
as one walk is what guarantees pose and Jacobian describe the same `q_pin`
— computing them separately could interleave with another query and mix
configurations.

`LOCAL_WORLD_ALIGNED` (cpp:55/75) is a Pinocchio reference-frame choice
worth understanding once: the Jacobian maps joint velocities to the
velocity *of the frame's origin point*, but expressed in *world (model
root) axes*. The alternatives are LOCAL (tool axes) and WORLD (spatial
velocity at the world origin — rarely what you want).

- `position_and_jacobian` keeps `topRows<3>()` — translation only.
- `pose_and_jacobian` keeps all six rows.
- **FLAG `Kinematics.cpp:45-82` | unnecessary** — neither free function has
  a caller in the run: the controller goes through `DualArmKinematics`'s
  own methods, which re-implement this pattern inline in
  `UpdateFullKinematics`. `RightPositionAndJacobian` (which probe_direction
  uses) also does not call these. They document the pattern, and tests
  exercise them, but in the control path they are duplicate code.

---

## DualArmKinematics — the 14-model / 7-controller adapter

The problem it solves (header comment h:87-97): the URDF models *both*
mounted arms — 14 velocity DoFs — but the controller and the Kortex
command path are seven-wide, right arm only. Also, Pinocchio represents
each *continuous* joint (Kinova joints 1/3/5/7, no position limits) not as
one angle but as the pair (cos θ, sin θ) — so the configuration vector has
22 entries (4 continuous × 2 + 3 bounded × 1, per arm). The adapter owns
all of that mapping so nothing else in the program ever sees a 22-vector.

The constants (h:105-110): `kArmDofs`=7, `kFullDofs`=14,
`kFullConfigurationSize`=22, and `kJointConfigurationSizes` =
{2,1,2,1,2,1,2} — per right/left joint, how many configuration slots it
occupies (2 = cos/sin pair, 1 = plain angle). **FLAG
`Kinematics.h:108-110` | edit-hazard** — this pattern is a hard-coded
mirror of which Gen3 joints are continuous; a URDF edit that adds limits to
joint 3 (say) changes Pinocchio's representation and this table, the
validation, and `SetJointAngle` all have to agree or the constructor
throws (best case) or writes angles into wrong slots (worst case — the
constructor's checks exist to force the best case).

### File-local helpers (Kinematics.cpp:95-179)

- `kRightJointNames` / `kLeftJointNames` (97-104) — the URDF joint names,
  in Kortex actuator order. The whole adapter maps **by name**, never by
  index order, so reordering joints in the URDF cannot silently permute
  the controller's joints.
- `ResolveJoints` (107-129) — for each of the seven names: check the joint
  exists, look up its Pinocchio joint id, verify its representation
  (`model.nqs[joint]` = expected 2-or-1 configuration slots,
  `model.nvs[joint]` = exactly 1 velocity slot), and record where its
  configuration (`idx_qs`) and velocity (`idx_vs`) variables live in the
  big vectors. Any mismatch throws with a message naming the joint.
- `ValidateCover` (131-163) — the paranoid completeness check: mark every
  configuration (or velocity) slot claimed by a right or left joint;
  throw on out-of-range, on any slot claimed twice, and on any slot
  claimed by nobody. Together with ResolveJoints this proves the 14 named
  joints tile the model's variables exactly — i.e. the URDF contains
  *only* the two arms, no stray extra joint the adapter would silently
  hold at neutral.
- `SetJointAngle` (165-178) — write one angle into the configuration
  vector in the joint's own representation: plain value for size 1,
  (cos θ, sin θ) for size 2. The size-2 branch is why continuous joints
  can never be given an out-of-range value — any angle maps onto the
  circle.

### Constructor (Kinematics.cpp:181-227) — validate everything, once

Called once in Main, *before any hardware session*, so a wrong URDF or
frame name fails the run before the arm is touched.

- **Lines 185-187** — initializer list: keep a reference to `dynamics`
  (the adapter does not own the model), stash the left nominal pose, and
  initialize `q_full_` to `pinocchio::neutral(model)` — the model's
  neutral configuration, which for cos/sin joints is (1, 0), a *valid*
  point on the circle. Starting from zeros instead would be an invalid
  configuration (cos²+sin²=0).
- **Lines 189-194** — hard check nq=22, nv=14: this binary only makes
  sense against the dual mounted URDF.
- **Lines 195-200** — both frame names (`base_link`,
  `EndEffector_Link` from Config.h) must exist.
- **Lines 202-209** — resolve both arms' joints and run both cover checks
  (q against 22, v against 14 with all-ones widths).
- **Lines 210-211** — cache the two frame ids.
- **Lines 213-226** — bake the left arm into `q_full_` once: per joint,
  require the nominal value be finite, and for *bounded* joints require it
  inside the URDF position limits, then write it via `SetJointAngle`. The
  left half of `q_full_` never changes again — the left arm is model-only
  (it shapes nothing in the right-arm Jacobian columns the controller
  reads, but it is part of the FK tree because the arms share a mount).

### `FullConfigurationForRight` (Kinematics.cpp:229-240)

Per call: require the measured right q be finite (a NaN would poison every
downstream pose), write the seven right angles into their resolved slots
(cos/sin expansion where needed), and return a reference to the member
`q_full_`. The header (h:125-126) warns the returned reference is
overwritten by the next call — do not store it. Public "for hardware-free
structural tests"; in the run it is only called through
`UpdateFullKinematics`.

### `UpdateFullKinematics` (Kinematics.cpp:242-269) — the per-cycle core

Runs on **every control cycle** (via Controller.cpp:47/83) plus the
startup-pose measurement. Everything in it must
stay allocation-free.

- **Lines 246-252** — assemble q_full, then the same one-walk pattern as
  the free functions: `computeJointJacobians` →
  `updateFramePlacements` → zero the workspace → `getFrameJacobian`
  (LOCAL_WORLD_ALIGNED, so model-root axes) for the right tool frame.
- **Lines 254-268** — re-express the Jacobian in **right-base axes**: take
  the right base frame's rotation, transpose it (inverse of a rotation),
  and rotate every column's linear and angular halves. The comment
  explains why there is no translation ("adjoint") term: the *point* whose
  velocity the Jacobian describes is unchanged (the tool origin) — only
  the axes it is expressed in change. **FLAG `Kinematics.cpp:254-268` |
  edit-hazard** — frame bookkeeping like this is where sign/order bugs
  hide: `base_R_world` is already the transpose, so "fixing" the transpose
  or reusing the loop for a *different point* (where the adjoint term
  would be required) silently produces a wrong Jacobian, which the
  controller would convert into wrong joint velocities on hardware.
  Note the loop rotates all 14 columns although only 7 are later selected
  — wasted work but harmless (the workspace is model-width by design).

### `RightPositionAndJacobian` (cpp:271-287) / `RightPoseAndJacobian` (cpp:289-305)

Both: `UpdateFullKinematics`, then compute the tool pose *relative to the
right base* — `base_M_tool = oMf[base]⁻¹ · oMf[tool]` (an SE3 = rigid
transform; the inverse-times product is "tool as seen from base") — then
**select** the seven right-arm columns out of the 14-wide workspace using
`right_v_indices_`, in Kortex actuator order. That column selection is the
7-wide/14-wide boundary in one place.

- `RightPoseAndJacobian` returns all six rows — the controller's per-cycle
  call and Main's startup-pose measurement use this.
- `RightPositionAndJacobian` returns only the translational 3×7 — used by
  the probe_direction tool, not by the run. Not flagged unnecessary: it is
  live tooling for the joint-6 situation, and it is the honest "position
  only" variant, but be aware only tools call it.
- **If the two `base_M_tool` computations drift apart** from
  `UpdateFullKinematics`'s axes convention, position and Jacobian stop
  describing the same frame — the invariant every header comment repeats.
  Duplicated four lines in two methods is the current cost of not having a
  shared private helper; an edit to one must be mirrored in the other.
  **FLAG `Kinematics.cpp:271-305` | edit-hazard** for exactly that
  duplication.
