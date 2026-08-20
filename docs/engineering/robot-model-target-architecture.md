# Robot model: target architecture

Status: design only, approved read-only 2026-08-20. No code changed.

## The split

    URDF + Pinocchio        where the robot IS
    joint_limits.yaml       where the robot is ALLOWED to move
    collision spheres       the robot's simplified physical VOLUME
    Scene + SDF             where OBSTACLES are

Each owns one question. Nothing owns two.

### URDF + Pinocchio own kinematics only

Joint axes, link transforms, mount to left_base/right_base, TCP and tool
frames, FK, Jacobians. Authoritative for all of these.

Not authoritative for joint limits. The URDF's fourteen `<limit>` tags are
silent on the four continuous joints (1/3/5/7) and, where they do speak, are
rounded away from Kinova Table 39: +-2.24 rad is 128.34 deg against 128.9,
+-2.57 is 147.25 against 147.8, +-2.09 is 119.75 against 120.3. Its velocity
figures are the hard limits with no derate. Reading them would be
duplication at a loss of accuracy. Leave them in place, but they are not a
source; a comment in the URDF should say so.

The DH tables GPMP2 needs are already generated from the URDF at build time
(`planning/CMakeLists.txt:131,145`; the controller depends on them via
`runtime/CMakeLists.txt:137`). There is no independent hand-written kinematic
model to remove.

### joint_limits.yaml owns limits

Single authoritative source for physical position and velocity limits.
Positions are Kinova Table 39; velocities are the hard limits the base
reports live at startup (80.0021 deg/s actuators 1-4, 70.004 deg/s 5-7).

On top of the physical numbers, ONE explicit margin mechanism, ordered by
layer, defined once in the same file:

    planner soft limit     = physical - planner_margin
    controller soft limit  = physical - controller_margin
    firmware error         outside both
    invariant: planner_margin > controller_margin > 0

Ordered rather than identical because the controller commands position from
integrated velocity and therefore always carries tracking error. If both
layers stopped at the same angle, a plan that legitimately reached its own
limit would sit exactly on the controller's trip line and any tracking error
would convert a valid plan into a stop. The gap IS the tracking allowance,
named and asserted rather than left to a min() chain.

Initial values, position, degrees:

    joint    physical   planner (-2)   controller (-1)
    2        128.9      126.9          127.9
    4        147.8      145.8          146.8
    6        120.3      118.3          119.3
    1/3/5/7  continuous - no limit, no margin

Velocity uses the same mechanism, expressed as a fraction because the
physical figure is itself a measured rate rather than a fixed stop:

    planner    0.93 of the live hard limit
    controller 0.95 of the live hard limit
    invariant: planner_fraction < controller_fraction

This replaces today's single 0.95 derate applied identically in both places.

### Copies to retire

Physical position limits exist in FIVE places today. All but the first must
become readers of it:

  1. planning/config/joint_limits.yaml            KEEP - the source
  2. control/Config.h kJointUpperDeg              read
  3. planning/optimisation/analytical_ik.h:26     read (currently +-2.2515,
                                                  2.5807, 2.0996 rad, which
                                                  disagrees with the source
                                                  in the third digit - the
                                                  IK seed and the planner do
                                                  not share a feasible set)
  4. the URDF <limit> tags                        ignore, comment
  5. firmware JOINT_LIMIT thresholds              DERIVE

On (5): `Connect::EnsureJointLimits` (runtime/Hardware.cpp:90) writes both
warn and error thresholds itself on every connection. The 118.0/123.0 on
joint 6 are ours, not Kinova-required, and `git log -S"118.0"` finds no
recorded reasoning. They should be derived from the same margin definition:
warn at the controller soft limit, error a stated distance beyond it.

Bench caveat: joint 6's configuration service has been wedged since
2026-08-04 and every config RPC to it times out, so the derived value will
not reach the firmware until that is fixed. EnsureJointLimits already logs
and continues per joint, so this degrades rather than blocks.

### Collision spheres stay hand-authored, re-anchored to links

Today `generateArmSpheres` (planning/optimisation/GenerateArmModel.cpp)
authors ~34 spheres against DH link indices, with along-link offsets scaled
by the ratio of live DH `d` values to a frozen `authored_d[7]` snapshot.
Deliberate hand-authoring is right and stays. The DH-ratio scaling is the
part to remove: it is a second interpretation of geometry.

Target: each sphere is declared as (link/frame NAME, local XYZ, radius) in
its own file, separate from the URDF.

Important constraint on how they reach GPMP2. A gpmp2 BodySphere is
(link_id, radius, centre relative to that link's base)
(third_party/include/gpmp2/kinematics/RobotModel.h:19-26), and the obstacle
factor calls sphereCenter(q, ...) WITH ITS JACOBIAN for every sphere at every
configuration during optimisation - that gradient is what pushes the
trajectory off the SDF. So the spheres cannot be transformed into mount once
before planning; that would freeze them at one configuration and destroy the
collision gradient.

Chosen approach: resolve the named frame and local offset into the DH link
frame ONCE at load, using Pinocchio, then hand GPMP2 the same
BodySphereVector it takes today. Explicit authoring, no second kinematic
interpretation, no change to the optimiser's inner loop. A build-time check
that the resolved offsets agree with Pinocchio's link geometry turns a URDF
change into a build failure rather than a silent sphere shift.

Rejected for now: a Pinocchio-backed FK adapter inside gpmp2's templated
RobotModel. It would remove the DH frame from the collision path entirely,
but needs per-sphere analytic Jacobians at optimiser speed and puts a heavier
FK in the hot loop.

### Scene and SDF stay separate

Wearer, other arm, tube, boxes, workspace obstacles -> occupancy grid -> SDF,
rebuilt per plan, paired with the arm model in the mount frame. MountSdf
already does this and `PlannerModel.h` already commits to mount as the
planner's only internal frame. Unchanged by this design.

## What is NOT in this design

- No RobotModel god-object. The four owners above stay four things.
- No change to which frame the planner works in.
- The 100-Hz/500-Hz split, the Cartesian controller and the SDF are untouched.

## Open question

Whether the 2 deg / 1 deg and 0.93 / 0.95 pairs are the right sizes. They are
placeholders chosen to be explicit, not measured. The right way to set the
controller margin is from observed peak tracking error on a logged run.

---

## Implemented 2026-08-20: the limits slice only

Done: joint_limits.yaml is the one physical table plus one `margins` block;
control/tools/generate_joint_limits.py turns it into a constexpr JointLimits.h
at configure and build time; Config.h, analytical_ik.h, quik_solveIK.h, the
panel and plot_run.py all read from it. The min() chain and
kJointSoftwareLimitMarginDeg are gone.

Deferred, on Christian's instruction: the firmware warn/error thresholds. See
the ordering hazard noted beside them in Config.h - with the min() removed,
j4 and j6 now warn before their software stop.

Not touched: planner algorithms, IK algorithms, collision spheres, the SDF,
validation logic, execution architecture.
