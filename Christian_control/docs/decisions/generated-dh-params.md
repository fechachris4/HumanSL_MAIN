# Generated DH parameters — the URDF is the only kinematic source

**Date:** 2026-08-05
**Status:** active

## Decision

`Christian_control/model/GEN3_dual_mounted.urdf` is the
single source of truth for all kinematics. Pinocchio parses it directly for
the controller and every project-owned FK/IK/Jacobian. The one consumer that
cannot eat a URDF — the vendored GPMP2 optimizer, whose `gpmp2::Arm` is
DH-parameter-only with no Pinocchio seam — gets its DH table **generated
from the URDF at build time** by
`planning/tools/generate_dh_params.cpp`. The generated
`dh_params_tool.yaml` lives only in the build tree; there is no committed
copy and hand-editing it is forbidden (it is overwritten on the next build).
The previously hand-authored YAMLs (`planning/config/
dh_params_tool.yaml`, `TrajectoryGeneration/config/dh_params.yaml`, root
`config/dh_params.yaml`) are deleted.

Rationale (Christian, 2026-08-05): a unified URDF makes frame
transformations trivially consistent across planner and controller, and the
previous arrangement already produced exactly the failure this prevents — a
hand-copied tool length (`d7 = -0.2574`) that matched no frame in the URDF
survived in `analytical_ik.h` until found by audit.

## How the generator works

The DH *structure* is a fixed frame convention, not measured data:
`a = 0` for every joint, `alpha = pi/2` for joints 1–6 and `pi` for
joint 7, `theta_offset = 0` then `pi` for joints 2–7, base = `Rx(pi)`
(the DH-root-in-base_link rotation). It must stay fixed because the
collision-sphere offsets in `GenerateArmModel.cpp` are expressed in the DH
link frames this convention defines. Only the seven link lengths `d1..d7`
are physical quantities, and with `a = 0` the tool position is **linear**
in them. The generator therefore:

1. verifies the convention's rotation chain against Pinocchio at 64 random
   configurations (rotation is d-independent),
2. solves the seven `d` values by least squares over 32 configurations
   (96x7 system, rank and conditioning guarded),
3. re-verifies full pose on 64 fresh configurations, and
4. writes the YAML atomically (temp + rename) with a DO-NOT-EDIT header —
   any failure exits non-zero and fails the build.

The URDF is a CMake `DEPENDS` of the generation step, so editing it
regenerates the YAML on the next build; `test_gpmp2_urdf_validation` then
independently checks the actual consumer chain (YAML file →
`createDHParams` → `gpmp2::Arm` FK) against Pinocchio at 0.1 mm / 0.01°.

## Tolerance floors (why not 1e-9)

The URDF writes its joint rotations to 5 significant figures (`rpy
"1.5708"`, `"3.1416"`), so exact-pi convention constants can only agree
with Pinocchio's parse to ~4e-5 rad / ~2e-5 m — that floor, not YAML
rounding, was most of the hand-YAML era's 0.0009° validation residual.
Generator thresholds (1e-3 Frobenius rotation, 2e-4 m position, 5e-4 rad)
and the tripwire (0.1 mm / 0.01°) sit ~4–10x above the floor; a genuinely
wrong convention constant overshoots by orders of magnitude. Rewriting the
URDF's rpy values at full precision would lower the floor but edits the
canonical hardware-facing model — deliberately not done here.

## Known parameter changes vs the hand table

The derivation recovers what the URDF actually says, which differs from
Kinova's published Table 94 in the 5th digit everywhere, and in one real
omission: `d6 ≈ -0.00035` (the URDF carries a ±0.000175 offset pair at
Actuator6/7 that the published table dropped). Planner geometry shifts
≤ ~0.5 mm, toward the URDF. Validation error collapsed from 0.44 mm
(hand-rounded values) to the ~0.02–0.05 mm floor.

## Collision spheres

`generateArmSpheres` takes the generated `d` vector; along-link sphere
stations are `authored_station x (d_current / d_authored)` (signed ratio,
authored denominators = the 2026-08-05 hand values), so a future tool or
link change moves the collision model with the links. Lateral (girth)
offsets stay absolute. Guards throw on a sign flip or a collapsed link
(|d| < 5 cm) — those mean the frame geometry changed and the layout must be
re-derived by hand, not silently rescaled. Known approximation: a longer
tool stretches the gripper sphere cluster proportionally rather than
translating it as a rigid body; if a substantially different tool ever
lands, consider flange-anchored stations
(`z' = z + (d7_new - d7_authored)`) for the gripper cluster instead.
