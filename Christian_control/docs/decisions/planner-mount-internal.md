# The planner is mount-internal; world exists only at its two edges

Date: 2026-08-20

Status: adopted and implemented.

## The decision

Everything inside the planner — IK, GPMP2, the SDF, validation — is
expressed in `mount`, the URDF root midway between the two arm bases.
`world_T_mount` (the immutable Vicon snapshot) appears in exactly two
places:

- **Input edge:** `ToMount` in `planning/src/PathFrames.h`, the single
  conversion module. Inputs may be declared in mount, left_base,
  right_base or world; a world-declared input requires a valid snapshot
  and is rejected per-request (`ToMountError`) without one.
- **Output edge:** `ProjectWorldTrajectory`
  (`planning/src/WorldTrajectoryProjection.h`), which carries the
  mount-frame result into the controller's world-frame wire contract,
  `WorldCartesianTrajectory`. That contract is unchanged and correctly
  named for its frame.

No frame conversion exists deeper than these two edges, and the SDF grid
is the static measured mount box (`MountSdf.h`) — it no longer depends on
where the rig sits in the room.

## Why

A mount-frame plan is meaningful relative to the robot. A world-fixed
task (holding the hand still in the room while the wearer moves) is the
CONTROLLER's job: it must continuously use the live `world_T_mount` to
update its Cartesian reference. GPMP2 stays mount-only. One conversion
site per direction is auditable; scattered transforms were the source of
past frame bugs (the ~0.12 m tool-vs-flange offset class).

## History — read the layers in this order

1. `planner-frame-mount.md` (2026-08-06): the frame then called "world"
   was renamed `mount`; the planner worked in it.
2. `docs/superpowers/specs/2026-08-15-world-cartesian-planner-controller-design.md`
   and `docs/superpowers/plans/2026-08-15-02-world-cartesian-planner-output.md`:
   Vicon integration made the planner world-INTERNAL. **Superseded** for
   planner internals by this note; their output-contract content
   (`WorldCartesianTrajectory`) still stands.
3. This note (2026-08-20): internals returned to mount, deliberately, as
   a canonical-frame migration; world remains only at the two edges.

## Deliberate non-choices

- No compile-time `MountPose` type yet: enforcement is API structure and
  tests, not a new type system.
- A world-declared axis-aligned obstacle box entering mount is replaced
  by its enclosing mount-frame AABB, inflation printed, never silent.
