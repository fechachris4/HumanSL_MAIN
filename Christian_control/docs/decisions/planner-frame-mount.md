# The URDF root is `mount`, and the planner works in it

Date: 2026-08-06

> **See also (2026-08-20):** `planner-mount-internal.md` — after a world-internal interlude (2026-08-15 Vicon design), planner internals are mount again, with world only at the two edges.

Status: adopted. The frame formerly called `world` is `mount`, and
`planner_bridge` expresses all internal geometry in it.

## Why the old name was wrong

`config/dual_arm_mounting.yaml` sets `mount_origin: base_midpoint`: the URDF
root sits exactly halfway between the two arm bases and is rigidly attached to
both by fixed joints. It is the rig. Calling it `world` implied a room frame
that does not exist anywhere in the model.

That mattered because this is a supernumerary-limb project. Once the rig is
worn, a room frame becomes real and must be called `world` — at which point
half the codebase would have meant the backpack by that name and half the
room. Renaming now costs a string sweep; renaming later would mean
disambiguating every existing use under a live ambiguity.

The confusion was already measurable, not hypothetical. `goal.yaml` carried a
standing warning that misreading its numbers as world "would move the target
0.709 m". `WorldSdf.h` described its own grid as *"the world-frame (base_link)
volume"* — two different frames named as one thing in a single parenthesis.
The 120 mm tool-vs-flange error found the same day was the same family of
mistake.

## What the planner used to do, and what it does now

The planner used to compute everything in the *controlled arm's own* base
frame, so a point had three spellings (mount, base_link/leftbase_link, and the
DH root, which is `Rx(pi)` from base_link). `BridgeMain` converted input into
whichever base frame `--arm` selected.

Now `PlannerModel` builds the gpmp2 arm at `DhRootInMount(left_arm)` =
`MountFromBase(left_arm) * DhRootInBaseLink()`, so FK, the collision spheres,
the goal and the SDF are all in `mount`. `BridgeMain`'s conversion collapses to
identity-or-one-step.

Nothing in `TrajectoryGeneration`'s FK/IK/Jacobian changed. `utils.cpp`
computes `base_pose * DhRootInBaseLink()^-1 * baselink_T_tool(q)`, so the two
`Rx(pi)` cancel and the result is `mount_T_tool` for any `base_pose`.
`createArmModel` was already frame-general and sphere offsets are link-local.

## One grid for both arms, not one per arm

An obstacle is a property of the world, not of an arm, and an `--arm both`
session is two sequential bridge processes reading the same `box:` from the
same `goal.yaml`. Per-arm grids would check that one box against a different
volume in each run — it could be accepted for one arm and refused for the
other, or accepted by both while lying outside one arm's grid in a region that
arm traverses, where gpmp2 contributes zero cost silently.

Cost was not the deciding factor and turned out not to be a factor at all: the
union is 57x69x58 = 228,114 cells against the old 60x60x59 = 212,400, about 7%
more. The reachable envelope is roughly spherical, so rotating it into `mount`
barely grows its bounding box.

`kGridCellM` stays 0.04. Changing resolution changes how obstacle cost is
sampled, which is a motion-path behaviour change and deserves its own
verification.

## The extents are measured, not chosen

`test_grid_coverage` unions the right (tool) and left (flange) collision
envelopes over a deterministic extremal sweep (bounded joints at
{lower, 0, upper} crossed with continuous joints at {-pi, -pi/2, 0, +pi/2})
plus 200k random configurations per arm, and prints a paste-ready constant
block when it fails. Measured 2026-08-06, in `mount`:

```
right  x [-1.052, 1.056]  y [-1.378, 0.654]  z [-0.952, 1.154]
left   x [-0.869, 0.870]  y [-0.478, 1.193]  z [-0.768, 0.971]
union  x [-1.052, 1.056]  y [-1.378, 1.193]  z [-0.952, 1.154]
```

The two differ because the left chain ends at a bare flange and carries no
gripper spheres, so neither envelope can be mirrored from the other. Resulting
margins are 0.062 to 0.088 m on every face, against a 0.05 m floor.

The deterministic sweep was added because uniform random sampling in seven
dimensions systematically under-covers fully extended poses — reaching one
needs several joints near an end of range at once.

## Two off-by-ones found on the way

**`WorldGridBounds()` overstated the volume by one cell.** gpmp2 interpolates
trilinearly, so the last usable coordinate on each axis is sample index `n-1`;
the function returned `origin + n*cell`. Consequences: the coverage test's
claimed 0.05 m margin was really 0.01 m, and the box check accepted boxes whose
top 4 cm lay in cells gpmp2 refuses to query. Fixed to `(n-1)*cell` before the
extents were re-derived, so they were not measured against one definition and
shipped against another.

**gpmp2's own range check is one sample wider than its interpolator.**
`convertPoint3toCell` admits a query landing exactly on `origin + (n-1)*cell`,
but `signed_distance()` then reads both `floor(idx)` and `floor(idx)+1`, so the
exact upper face indexes past the end. `WorldGridBounds()` returns gpmp2's
accepted range (that is what will and will not throw) and `BoxWithinGridBounds`
compares the upper faces **strictly**, so nothing we ship can land there. The
arm is kept 0.05 m clear of it by the coverage test regardless.

## Obstacle boxes: accepted in either frame, inflated when converted

A box declared in an arm's base frame is axis-aligned *there*, and the bases are
rolled 1.2085 rad (69.2 deg) from `mount`, so the rotated box is not
grid-aligned. Rather than refusing it — which would hand the operator a
coordinate conversion in the one file they hand-edit before a run — the rotated
shape is replaced by its enclosing axis-aligned box, `|R| * half_extent`. That
is larger than requested, never smaller, which is the safe direction for
avoidance, and the inflation is printed rather than applied quietly.

## `planner.yaml` was not re-authored

`goal.position_sigma_xyz` looks like a base_link-axes bake-in and is not.
gpmp2's `GaussianPriorWorkspacePose::evaluateError`
(`third_party/include/gpmp2/kinematics/GaussianPriorWorkspacePose.h:70`) returns
`des_pose_.logmap(actual)` = `Logmap(des_pose^-1 * actual)`. Left-multiplying
both poses by the same `T_mount_base` cancels exactly, so the six-vector error
is bit-identical. The sigma axes are the goal pose's own body axes, never the
model frame's — the "y is ten times looser" note was always about the goal's y.

## Evidence

- Rename verified substitution-only: every removed line, after applying the
  same substitutions, reappears among the added lines; the URDF's numeric
  tokens are byte-identical before and after; `test_dual_arm_mounting` prints
  unchanged numbers (0.113415, 1.208500, 0.149613). Pinocchio accepts a root
  link named `mount`.
- `test_planner_model` now asserts `ToolPoseInMount == MountFromBase * adapter`
  at 1e-6 for **both** arms — strictly stronger than before, since it pins the
  mounting composition and the sign of the +-1.2085 roll. The left arm's
  mirrored mount was previously untested in the planner. An independent anchor
  asserts the two base origins are exact negatives, which a right-hand
  transform copy-pasted into the left slot would fail while satisfying every
  consistency check.
- `test_bridge_main`'s out-of-grid box case now asserts the diagnostic
  substring. It previously passed for the wrong reason: a mount-frame box hit
  the old wrong-frame refusal and never reached the grid check at all.
- End-to-end behaviour neutrality: the same physical goal from the same start
  (`runs/2026-08-06/loop_log_left_20260806_184312.csv`, goal `left_base
  [0.0239, -0.563535, 0.0235506]`) solved to **1.10 mm before** the move and
  **1.09507 mm after**, exit 0 both times. The plans are not bit-identical —
  the optimiser's linearisation points move even under an exact rigid
  transform — which is what behaviour-neutrality means for a nonlinear solve.
- `ctest`: planner_bridge 10/10, basic_control 11/11, unchanged from the
  pre-work baseline.

## Where a room frame goes

`T_room_mount`, identity while the rig is bolted to a bench and supplied by
motion capture once it is worn. It composes ABOVE `mount`, in `BridgeMain`'s
frame boundary and in `DualArmKinematics::MountFromBase`. It is documented
rather than stubbed: an identity function nothing calls is clutter, and the
rename is what actually reserves the name.

Note for that day: the wearer's keep-out volume belongs in a **torso** frame,
not the room frame. Expressed in the torso, the wearer's motion cancels in the
distance Jacobian and only the arm's own joints appear; expressed in the room,
every step looks like the obstacle moving. The MSc simulation project's
`controller/human_safety.py` already works this way and records the reasoning.

## Not addressed here

- Non-deterministic planning (`analytical_ik.h:460` seeds `std::mt19937` from
  `std::random_device`, plus bare `rand()` in `TrajectoryInitiation.cpp`), so
  the same request can produce different plans. Frames are not implicated.
- `SolveToPosition` still returns `ok` regardless of `final_goal_error_m`.
- `TrajectoryExecution/src/Dynamics.cpp`'s `R_world_to_base_ = Rx(pi)`, a
  second and contradictory mounting assumption that ignores the URDF. Dead (no
  caller on any live path) but compiled into `bridge_core`.
- `utils.cpp`'s `world2base()` (zero callers) and `createArmBasePoses` (already
  marked dead, contradicts the URDF). Both are more confusing after the rename.
- `scripts/show_frames.m` still documents a `WORLD x y z` pipe target grammar
  deleted in `stage2-joint-trajectory-following.md`.
