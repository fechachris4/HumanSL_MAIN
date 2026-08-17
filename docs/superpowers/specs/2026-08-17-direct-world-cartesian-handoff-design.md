# Direct World-Cartesian Handoff Design

**Date:** 2026-08-17  
**Status:** Approved for implementation by Christian in the current session  
**Scope:** Replace production planner-to-controller FIFO/file IPC while
preserving the existing `WorldCartesianTrajectory` boundary.

## 1. Decision

The production planner worker will run inside the `controller` process as a
non-real-time worker thread. The planner will receive the existing typed
`PlanningRequest` snapshot and return one typed
`std::unique_ptr<WorldCartesianTrajectory>`. Ownership will move directly into
the existing `CartesianTrajectoryMailbox`:

```text
500 Hz controller measurement
    -> PlanningRequestSlot (existing latest-value snapshot)
    -> in-process planner worker (outside the cyclic thread)
    -> std::unique_ptr<WorldCartesianTrajectory>
    -> CartesianTrajectoryMailbox::Publish(std::move(trajectory))
    -> existing CartesianReferenceSource / execution core
```

The planner remains the source of truth for trajectory construction and
planner-side validity. The controller continues to own execution, hardware
communication, and runtime state-dependent activation and shutdown behaviour.

The controller and planner are currently separate executables, so a direct
typed function call cannot be added without moving the planner worker into the
controller process. Unix sockets, anonymous pipes, shared memory, and other
process transports were rejected for the production path because they would
retain serialization, framing, parser code, and a second ownership boundary.

## 2. Preserved motion contract

The handoff remains the current world Cartesian contract; it does **not** revert
to a joint-trajectory controller boundary.

Each trajectory point retains:

* `t_from_start_s` in seconds, strictly increasing from zero;
* world-frame TCP position in metres;
* world-frame unit quaternion in `x y z w` order;
* world-frame linear velocity in metres/second;
* world-frame angular velocity in radians/second;
* the final-point arrival eligibility flag;
* the planner trajectory identity and Vicon sequence provenance.

GPMP2 still solves in joint space internally and the planner still performs the
dense FK/Jacobian projection into `WorldCartesianTrajectory`. No planned joint
posture crosses into the controller. The planner worker never runs in the
500 Hz loop; the controller's existing measurement, reference, control law,
velocity limiting, position integration, Kortex exchange, watchdog, fault, and
teardown paths remain the execution path.

## 3. Ownership and lifetime

* `PlanningRequestSlot` remains the latest-value, fixed-size snapshot from the
  cyclic controller-side code to the planner worker. It is copied as a value;
  no planner pointer or mutable controller state crosses the thread boundary.
* The planner constructs one `WorldCartesianTrajectory` and transfers it with
  `std::unique_ptr` to `CartesianTrajectoryMailbox::Publish`.
* The mailbox owns an unread candidate until the control thread atomically
  takes it. Replacing an unread candidate retires the displaced allocation
  outside the cyclic thread.
* The control thread owns an active trajectory while it is being interpolated.
  Consumed, rejected, completed, or cancelled trajectories use the existing
  non-allocating retire path and are reclaimed outside the cyclic thread.
* A planner result completed after shutdown begins is discarded by the stop
  check or mailbox teardown; no result may outlive the mailbox, kinematics, or
  planner worker that owns it.

No generic event bus, registry, factory, singleton, or new message framework is
introduced. The handoff consists of the existing concrete request slot, the
existing concrete trajectory type, and the existing concrete mailbox.

## 4. Validation and runtime guards

`ProjectWorldTrajectory` and the planner's existing path/joint/model checks
remain the single planner-side construction/validity path. A successfully
returned typed trajectory has already passed the planner's required
projection/contract checks before publication.

The controller will no longer repeat the structural
`ValidateWorldCartesianTrajectory` call solely to re-check planner semantics.
It will retain only checks that require live controller state or are needed for
safe operation:

* non-null and valid object lifetime;
* trajectory provenance against the current world sequence and replan floor;
* first-pose continuity against the currently measured world TCP pose;
* stale-world, completion, cancellation, and re-anchor state transitions;
* existing non-finite, following-error, joint-boundary, watchdog, fault,
  servoing, and teardown behaviour.

These checks do not make the controller a second planner: they protect the
handoff and execution against live-state changes that the planner cannot know
at solve time.

## 5. Process and UI changes

The production session will start one `controller` process. It will no longer
start `planner_bridge --serve` as a second process or wait for planner FIFOs,
publication marker files, or planner worker PIDs.

`run_session.sh` will retain the existing operator gates, binary freshness
checks, run directory, goal/planner configuration snapshots, controller log,
Enter/Ctrl+C stop path, and session provenance. It will wait for the controller
run/log state and typed planner activation evidence rather than for FIFO
creation and `published_*.ok` files.

The panel's offline **SOLVE AND PREVIEW** action is outside the production
planner-to-controller path. It may continue to invoke the standalone planner
CLI so the UI can preview a plan without hardware; that CLI output is not used
by the controller. Production controller execution must contain no trajectory
serialization/deserialization, FIFO polling, or temporary trajectory files.

## 6. Source-level migration shape

Expected production changes:

* factor the planner solve/projection from `RunBridge` into a typed callable
  usable by both the in-process worker and the offline preview wrapper;
* add the smallest concrete in-process planner-worker loop, consuming
  `PlanningRequestSlot` and publishing to `CartesianTrajectoryMailbox`;
* link the planner implementation and its required GPMP2/Pinocchio libraries
  into the controller executable only at the non-real-time boundary; keep the
  `humansl_execution_core` archive hardware-independent;
* start and join one worker per selected arm in the controller lifecycle;
* remove controller planning-request FIFO creation/writer code;
* remove controller trajectory FIFO creation/input/parser adapters;
* remove planner-service FIFO reader/writer, string trajectory publication,
  publication markers, and production-only artifact plumbing;
* update launcher, panel session status, tests, and documentation to describe
  the single-process production topology.

The exact file list will be finalized during the implementation plan after the
current source has been edited only through the approved migration slice. No
unrelated panel, configuration, controller-law, or safety cleanup is included.

## 7. Characterisation and acceptance tests

Before deleting the old path, add or identify hardware-free evidence for:

1. planner output values, point count, ordering, frames, units, timestamps,
   and provenance are unchanged;
2. a direct move transfers the exact trajectory object values into the
   controller mailbox without serialization or resampling;
3. the controller executes a trajectory at most once;
4. an unread trajectory is safely replaced/reclaimed;
5. a trajectory remains valid after the planner function returns;
6. planner shutdown, controller shutdown, and destruction order do not crash;
7. a planner result that completes during shutdown is discarded safely;
8. no 500 Hz `Step`/`ResolveStop` path blocks, allocates, or invokes GPMP2;
9. the single-process launcher/UI path reaches the same execution entry point;
10. the old FIFO symbols, paths, parser functions, and production launch steps
    are absent after removal.

Run the existing basic-control and planner-bridge suites before and after the
migration. Physical motion, Vicon calibration, world tracking, collision
clearance, and person-nearby safety remain unproven until a separately
authorized supervised hardware session.

## 8. Explicit non-goals

This migration does not:

* change the world Cartesian trajectory equations or controller gains;
* change joint limits, velocity limits, following-error policy, or fault policy;
* move GPMP2 into the cyclic thread;
* introduce joint-trajectory execution into the controller;
* add automatic trajectory modification, clipping, resampling, or replanning
  policy at the controller boundary;
* run a robot or claim physical equivalence from offline tests.

