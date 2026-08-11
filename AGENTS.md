# AGENTS.md

## Project purpose

This repository implements the control system for a wearable
Supernumerary Robotic Limb using Kinova Gen3 arms.

The intended control objective is to maintain or track the end-effector
pose in the world frame while the wearer and backpack-mounted robot base move.

## Verified current state

- GPMP2 currently produces a timed joint trajectory:
  q_ref(t), qdot_ref(t).
- That trajectory is a reference, not a final hardware command.
- The active joint controller computes:
  qdot_raw = qdot_ref + Kp * wrap(q_ref - q_measured).
- A resolved-velocity Cartesian PD controller is implemented and tested.
- The Cartesian controller includes pose feedback, measured-twist damping,
  reference-twist feedforward, damped least-squares inverse kinematics and
  null-space joint-limit avoidance.
- The Cartesian controller is currently unwired because production code does
  not produce PoseReference values.
- Both controller laws are intended to pass through the same velocity limits,
  joint-boundary handling, integration and hardware safety path.

## Current architecture objective

We are deciding how to expose the complete command-generation pipeline clearly:

reference
-> feedback control law
-> raw joint velocity
-> velocity and joint limits
-> position-command integration
-> hardware command.

Do not implement architecture changes until Christian explicitly approves
the target architecture.

## Engineering constraints

- Do not delete the Cartesian controller or its tests.
- Do not classify code as obsolete only because it is currently unreachable.
- Do not introduce managers, services, registries, factories or plugin systems.
- Prefer plain structs, pure functions and explicit control flow.
- An abstraction must either replace existing complexity or have at least two
  real current implementations.
- GPMP2 must remain outside the 500 Hz control loop.
- Blocking Vicon, file, terminal and logging operations must remain outside
  the 500 Hz control thread.
- Safety and actuation must not be bypassed by either controller mode.
- Frame names, units, timestamps and reference frames must be explicit.
- Do not run robot-facing commands unless Christian explicitly authorizes them.

## Working protocol

For requests to inspect, review, discuss or plan:

- Read the relevant source.
- Report evidence using files, symbols and call paths.
- Do not edit code.

For implementation requests:

- Implement only the explicitly approved migration slice.
- Before editing, state which files will change and why.
- Preserve behaviour unless a behaviour change is explicitly requested.
- Run all relevant hardware-free tests.
- Report added and removed production files, classes and concepts.
- Do not commit, push or operate hardware unless explicitly requested.
