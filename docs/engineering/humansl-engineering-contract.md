# HumanSL SRL Engineering Contract

**Status:** Binding
**Applies to:** Motion control, Vicon integration, kinematics, GPMP2 planning, low-level command generation, simulation, testing and experimental claims.

This contract governs the photographed dual-arm SRL: two Gen3 arms on separate custom shoulder mounts, attached to a torso rig with overhead support, with Vicon segments intended for the Mount, both bases and both end effectors.

A conflicting implementation must not proceed until the conflict is documented and explicitly approved.

## 1. Describe the physical system truthfully

The current photographed configuration shall be described as a:

> **Supported torso-mounted dual-arm SRL rig intended for wearable operation.**

It must not be described as human-worn validation unless a person actually wears the system during the reported experiment.

Every hardware result must record:

* Rigid fixture, mannequin-supported or human-worn condition.
* Whether the overhead suspension is attached.
* Whether the suspension is slack, partially load-bearing or constraining motion.
* Left-arm, right-arm or simultaneous operation.
* Payload, tool and marker configuration.

Simulation, supported-rig testing and human-worn testing are separate evidence levels.

## 2. Frames and physical points are defined before control code

Use the convention:

```text
A_T_B = pose of frame B expressed in frame A
```

It maps coordinates from frame `B` into frame `A`.

The authoritative controller frame chain for arm `i`, where `i` is explicitly `left` or `right`, is:

```text
world_T_base[i]
    = world_T_mount_segment
    * mount_segment_T_base[i]

world_T_tcp[i]
    = world_T_base[i]
    * base_T_tcp[i](q[i])
```

Hard rules:

* `mount_segment_T_base[left]` and `mount_segment_T_base[right]` are separate six-degree-of-freedom transforms.
* They must not be assumed identical, mirrored or equal to identity without physical evidence.
* The custom shoulder brackets visible on the rig are not automatically represented by the standard arm model.
* `base_T_tcp(q)` must include the actual controlled tool point, including any flange-to-adapter and adapter-to-gripper offset.
* The Vicon segment origin is not automatically the marker centroid, robot base, flange or TCP.
* A marker centroid must not be used as a robot frame unless a fixed calibrated transform gives it physical meaning.
* Translation and rotation must not be combined into one scalar without a declared weighting.
* Orientation error must use a declared rotation convention. Euler-angle subtraction is not an implicit default.

Control units are metres, radians and seconds.

Vicon millimetres and robot-interface degrees are converted once at their respective adapter boundaries.

## 3. Christian owns the physical definitions

Implementation agents must not invent or infer the following from photographs, CAD, variable names or apparent symmetry.

Christian must personally confirm:

1. Which physical markers belong to `Mount`, `LeftBase`, `RightBase`, `LeftEE` and `RightEE`.
2. Which rigid physical component each segment follows.
3. That each base marker cluster is attached to the non-moving base, not a rotating first link.
4. That each end-effector cluster follows the intended wrist or tool body.
5. The exact physical TCP position and axes for each gripper.
6. The Vicon world-axis directions and signs.
7. The left and right base-frame orientations.
8. Whether the overhead suspension affects Mount movement or load.
9. Whether any marker, printed collar, bracket or cable moves relative to the body it is intended to track.
10. Whether the current experiment represents a supported rig or a wearable system.

If one of these facts is unknown, the agent must identify it as unresolved. It must not silently choose a value.

## 4. Measured, estimated, referenced and commanded quantities remain separate

Names and logs must preserve the distinction between:

* **Measured:** Vicon segment poses, joint feedback and robot feedback.
* **Estimated:** Mount twist, base pose, TCP pose and TCP twist.
* **Referenced:** Desired Cartesian pose, desired Cartesian twist and planned trajectory.
* **Commanded:** Requested joint velocity, limited joint velocity and integrated joint position command.
* **Simulator truth:** Exact simulated state used only for simulation evaluation.

Recommended suffixes are:

```text
*_meas
*_est
*_ref
*_cmd
*_gt_sim
```

Simulator truth must never enter production controller calculations.

The robot's own reported tool pose is model-based feedback. It is not an independent physical ground truth.

Vicon is an independent measurement system, but it is not perfect ground truth. Its calibration, segment definition, occlusion and latency must remain visible in the analysis.

## 5. Base and end-effector Vicon segments are held-out validation by default

The intended production state estimate uses:

* The `Mount` Vicon segment.
* Joint encoder feedback.
* The calibrated Mount-to-base transforms.
* The robot kinematic model.

The following are validation measurements by default:

```text
LeftBase
RightBase
LeftEE
RightEE
```

Direct base validation is:

```text
world_T_base_val[i]
    = world_T_base_segment[i]
    * base_segment_T_base[i]
```

Direct TCP validation is:

```text
world_T_tcp_val[i]
    = world_T_ee_segment[i]
    * ee_segment_T_tcp[i]
```

The disagreement between the Mount-derived estimate and direct Vicon measurement is a **base-chain residual** or **TCP-chain residual**.

It must not initially be called Mount flex, because it may also contain:

* Extrinsic calibration error.
* Marker motion.
* Segment-origin error.
* Vicon noise.
* Tool calibration error.
* Structural flex.
* Backpack or fixture slip.

Using the base or end-effector segments as online controller inputs is a **C: controller behaviour change** and requires explicit approval.

## 6. The moving-base physics has one authoritative meaning

For each arm:

```text
world_V_tcp
    = world_V_base_at_tcp
    + world_J(q) * qdot
```

The base-induced TCP linear velocity includes the rotational lever arm:

```text
world_v_tcp_from_base
    = world_v_base
    + world_omega_base
      cross world_r_base_to_tcp
```

Therefore:

* Mount rotation cannot be treated as translation only.
* The same Mount linear velocity cannot simply be copied to both bases.
* Left and right bases have different Mount-to-base lever arms.
* Every twist must state its expression frame and physical reference point.
* Every Jacobian must state its expression frame, row ordering and reference point.

The implementation may either:

1. Transform the desired world pose into the current base frame; or
2. Form the world-frame twist request and subtract the base-induced TCP twist.

The selected derivation must be written before code. The same base motion must not be compensated twice.

## 7. World hold and planned tracking are distinct reference cases

For world hold:

```text
world_T_tcp_des[i]
    = world_T_tcp[i] at engagement time
```

The desired world pose remains fixed until an approved reference change.

For planned tracking, every plan must declare:

* Arm identity.
* Pose frame.
* TCP frame.
* Position and orientation units.
* Time origin.
* Pose samples.
* Twist samples, where supplied.
* Mount state or base assumption used during planning.
* Planner start and completion timestamps.

Planned pose and twist must represent the same trajectory.

A planner failure must not be represented as an executable trajectory.

Plan rejection, automatic blending, automatic replanning, trajectory slowing or target modification are **C** unless already approved.

## 8. The 500 Hz execution path is isolated

The control execution core consumes one timestamped cycle snapshot and produces commands plus diagnostics.

It must not directly depend on:

* MuJoCo.
* Kortex.
* The Vicon SDK.
* GPMP2.
* File I/O.
* Terminal I/O.
* The operator panel.
* Network calls.
* Blocking locks.
* Sleeps.
* Unbounded work.

Vicon acquisition and GPMP2 optimisation remain outside the approximately 500 Hz path.

Every Vicon sample supplied to the core must carry:

* Source frame number.
* Measurement or source timestamp where available.
* Receive timestamp.
* Validity.
* Occlusion state where available.
* Age at the control cycle.

A 500 Hz controller must not describe a reused 100 Hz Vicon sample as a 500 Hz Mount measurement.

Changing filtering, prediction, stale-data handling, re-anchoring or recovery behaviour is **C** unless already approved.

## 9. Low-level position command generation preserves separate states

The following must remain separate:

```text
qdot_requested
qdot_applied_after_existing_limits
q_position_command
q_measured
qdot_measured
```

Joint-position integration must use the measured loop interval or a documented bounded timing assumption.

Existing command limits, robot faults, emergency stops, following-error protections and laboratory procedures remain authoritative.

This contract does not create new hardware safety thresholds.

## 10. Diagnostics must not silently become behaviour

Every proposal is classified as:

| Class | Meaning                        |
| ----- | ------------------------------ |
| **A** | Diagnostic only                |
| **B** | Experiment pass/fail criterion |
| **C** | Controller behaviour change    |
| **D** | Operator warning               |
| **E** | Hard safety stop               |

Default classification is **A**.

The following are **A by default**:

* Manipulability.
* Jacobian condition number.
* Minimum singular value.
* Damped-inverse residual.
* Constrained task residual.
* Joint margin.
* Saturation ratio.
* Number of limited cycles.
* Vicon sample age.
* Plan age.
* Inter-arm distance.
* Wearer or mannequin clearance.
* Base-chain residual.
* TCP-chain residual.
* Estimated Mount flex.
* Null-space leakage.
* Following-error diagnostics outside existing approved protections.

An **A** metric must not automatically:

* Alter a command.
* Change posture.
* Slow a trajectory.
* Reject a plan.
* Re-anchor world hold.
* Issue a warning.
* Stop the robot.

Any such action is **C**, **D** or **E** and requires a separate approved proposal.

Existing planner collision constraints and existing hardware protections are preserved. Changing them is not authorised by this contract.

## 11. Dual-arm interaction and Mount mechanics must be measured

Left and right arms must never be identified only by array order, connection order or filename order.

The system must record:

* Explicit left and right identities.
* Shared experiment or coordination identifier.
* Command start times for both arms.
* Relative start-time error.
* Left and right base-chain residuals.
* Left and right TCP errors.
* Relative left-to-right base motion.
* Inter-arm and torso-clearance diagnostics.
* Vicon validity for every tracked segment.
* Support and suspension condition.

The central Mount and both shoulder assemblies must not be assumed rigid because they appear connected.

Rigidity should be evaluated using the relative transforms:

```text
mount_segment_T_left_base_segment(t)
mount_segment_T_right_base_segment(t)
left_base_segment_T_right_base_segment(t)
```

Their variation is **A** unless used as a predeclared **B** experimental outcome.

Online flex compensation is **C**.

## 12. Tests must be capable of detecting wrong physics

Tests should use an independent oracle where feasible.

A test must not calculate its expected result through the same transform helper, FK function or frame chain being tested.

Required motion-related tests include, where applicable:

* Known translation along each Vicon world axis.
* Known rotation about each world axis.
* Fixed-Mount arm movement.
* Fixed-joint Mount movement.
* Mount rotation with an extended arm to expose the rotational lever-arm term.
* Left-only and right-only asymmetric motion.
* Simultaneous dual-arm motion.
* Direct base-segment versus Mount-derived base comparison.
* Direct EE-segment versus Mount-plus-FK TCP comparison.
* Planned pose-versus-twist consistency.
* Commanded-versus-measured joint response.

Required mutation tests include deliberately introducing:

* Millimetre/metre error.
* Degree/radian error.
* Inverted transform.
* Wrong transform multiplication order.
* Left/right arm swap.
* Missing flange-to-TCP offset.
* Wrong orientation sign.
* Omitted rotational lever-arm term.
* Time-shifted Vicon data.

The evaluation must fail clearly under these mutations.

Existing behaviour must be characterised and saved before refactoring.

## 13. Simulation and physical evidence must be labelled correctly

Simulation may establish:

* Equation and sign consistency.
* Frame and unit correctness.
* Behaviour under an explicit model.
* Sensitivity to delay, noise and saturation.
* Whether mutation tests detect incorrect implementations.

Simulation cannot establish:

* Physical Mount rigidity.
* Physical TCP calibration.
* Vicon accuracy on this rig.
* Actual low-level arm response.
* Cable forces.
* Backpack or fixture slip.
* Wearer comfort or clearance.
* Human-worn performance.

Every reported result must be labelled as one of:

```text
unit test
simulation
recorded-data replay
supported torso-rig hardware test
human-worn hardware test
```

Simulation results must never be described as physical proof.

## 14. Abstractions and comments must remain physically useful

Generic managers, factories, services, registries and framework layers are prohibited unless they replace demonstrated existing complexity.

There must be one clear owner for:

* Frame transforms.
* Measured state.
* Estimated state.
* Reference state.
* Planner output.
* Existing safety state.
* Command output.

Simulation and hardware should use the same control mathematics through different boundary adapters.

Comments explain:

* Physical assumptions.
* Frame conventions.
* Units.
* Calibration provenance.
* Timing assumptions.
* Safety decisions.
* Why a non-obvious equation is valid.

Comments must not narrate syntax or preserve obsolete design history.

## 15. Definition of done

A motion-affecting change is complete only when:

* Its physical question is stated.
* Its equations are written before implementation.
* Every frame, point, unit and sign convention is declared.
* Measured, estimated, referenced and commanded quantities are separated.
* Its A–E classification is recorded.
* Relevant independent and mutation tests pass.
* Existing hardware safety behaviour is unchanged unless separately approved.
* The 500 Hz path remains free of forbidden dependencies.
* Timing impact is measured or bounded.
* Simulation and physical evidence are distinguished.
* Any required physical fact has been personally confirmed by Christian.
