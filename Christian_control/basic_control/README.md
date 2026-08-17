# basic_control

`basic_control` is the only arm-moving program in HumanSL. It controls either
Kinova Gen3 arm at 500 Hz and has one production control law: world-frame
Cartesian pose/twist tracking. GPMP2 joint posture never enters this process.

## Command path

Each cycle follows one explicit path:

```text
Vicon T_W_M and V_W_M + measured q/qdot
  -> measured T_W_E, J_W and V_W_E
timed WORLD pose/twist reference
  -> Cartesian pose/twist error
  -> damped resolved-rate IK + joint-limit null-space term
  -> raw qdot
  -> per-joint velocity limits and joint-boundary handling
  -> persistent position-command integration
  -> Kortex cyclic POSITION command
```

The law is the C++ port of the Python simulation law:

```text
e_p = p_W_E,d - p_W_E
e_R = log(R_W_E,d R_W_E^T)
e_V = V_W_E,d - V_W_E
V_task = Kp [e_p; e_R] + Kd e_V
qdot_raw = J_W^T (J_W J_W^T + lambda^2 I)^-1 V_task + qdot_null
```

There is no separate base-velocity feedforward term. Filtered Vicon Mount
twist is transported to the end-effector point and included in the measured
`V_W_E`; planned Cartesian twist enters only through `e_V`. This is important:
base motion is measured feedback, and it is counted exactly once.

The current transform chain is
`T_W_B = T_W_M * T_M_B` and `T_W_E = T_W_B * T_B_E`. `T_W_M` is the tracked
Vicon `Mount` segment pose. The fixed `T_M_B` comes from the authoritative
mounted dual-arm URDF. A separate marker-cluster-to-model-Mount calibration is
not yet represented; the tracked Mount segment is presently assumed to be the
model Mount frame.

## Execution core

Since the Plan 01 extraction (2026-08-17) the per-cycle command pipeline —
measurement, world-freshness classification and stale-twist decay, the
Cartesian reference, the control law, the non-finite hold, the velocity
clamp, and position integration — is composed by one hardware-independent
class, `ArmExecutionCore` (`src/ExecutionCore.h`). It is built into the
static library `humansl_execution_core` together with the immutable
`ExecutionConfig` snapshot, the controller/reference/actuation
mathematics, the Pinocchio kinematics, and the Cartesian contracts. The
library links only the bundled Eigen/Pinocchio and pthread; the registered
`execution_core_linkage` test fails if a Kortex or Vicon SDK symbol enters
the archive. The Kortex runner keeps takeover T1-T6, the cyclic exchange,
typed planner handoff, logging, and teardown: each cycle it assembles one
`ArmExecutionInput`, calls `core.Step`, publishes a fixed-size planning
request only on a replan edge, transmits the returned command frame, and only
then asks `core.ResolveStop` for the stop verdict, so the original
send-then-resolve order is preserved.

Validation status: this refactor is **offline-validated only**. The
evidence is unit tests plus recorded-data replay — the frozen
pre-extraction characterization fixture
(`tests/fixtures/execution_preextract_v1.csv`) replays through the
extracted core with byte-identical discrete fields — which establishes
software equivalence on recorded inputs, not physical equivalence. The
refactored `controller` binary has not been run on the arm. The supervised
revalidation procedure is
`Christian_control/docs/runbooks/execution-core-hardware-revalidation.md`;
it is a procedure, not an authorization.

## Planner boundary

`planner_bridge` still solves in joint space internally with GPMP2. After the
final plan passes validation and timing, the planner application projects every
dense state through Pinocchio FK and `J_W qdot`. The production handoff is one
typed `WorldCartesianTrajectory` object containing world-frame pose/twist
samples; its offline text formatter is used only by the standalone preview
CLI.

The trajectory contains no `q_ref`, `qdot_ref`, null-space posture, or frame selector.
The planner constructs and validates the complete typed object. The controller
checks only live provenance and first-pose continuity before activation; it does
not parse or re-run planner trajectory validation.

Planning is asynchronous inside the same controller process. On startup and
after a prolonged Vicon dropout, the 500 Hz loop copies one fixed-size request
containing measured `q`, `T_W_M`, and Vicon provenance into a wait-free slot.
One non-real-time planner worker per arm consumes that slot, solves, and moves
the resulting `WorldCartesianTrajectory` directly into the controller mailbox.
GPMP2 never runs in the cyclic thread; requests arriving during a solve
coalesce to the newest pending request, and solves remain sequential per arm.

## Vicon dropout behavior

- Fresh means a valid Mount sample no older than 50 ms.
- During a brief stale interval, the world pose and Cartesian reference are
  held, reference time pauses, and the last measured Mount twist decays as
  `exp(-stale_duration/tau)`. Stale duration covers both old source frames
  and continued frames in which the Mount segment alone is invalid.
- Fresh recovery before 200 ms resumes the same trajectory without wall-clock
  catch-up.
- At 200 ms continuously stale, the active trajectory is cancelled once.
  Recovery captures the current measured world pose, rejects pre-gap plans, and
  issues one fresh planning request.

No dropout branch bypasses command limits, following-error checks, fault checks,
joint-boundary handling, or servoing teardown.

## Source map

| File | Responsibility |
| --- | --- |
| `State.h` | fixed robot, measured Cartesian, reference, and status records |
| `Frames.h` | world transform, Jacobian rotation, and twist transport math |
| `Controller.cpp` | the sole Cartesian error and resolved-rate law |
| `CartesianReference.cpp` | startup hold, interpolation, splice, completion, and dropout states |
| `CartesianTrajectoryMailbox.cpp` | typed trajectory ownership handoff and reclamation |
| `InProcessPlanner.cpp` | non-real-time typed planner worker |
| `PlannerRuntime.cpp` / `BridgeMain.cpp` | typed planner solve and projection runtime |
| `ExecutionCore.cpp` | `ArmExecutionCore`: the hardware-independent per-cycle pipeline |
| `ExecutionConfig.cpp` | immutable configuration snapshot taken once from `Config.h` |
| `Runner.cpp` | 500 Hz timing, Kortex takeover/exchange, and teardown around the core |
| `Actuation.cpp` | velocity-limited position integration and joint boundaries |
| `Safety.cpp` / `StopPriority.h` | readiness, fault decoding, stop precedence, servoing restoration |
| `Hardware.cpp` | Kortex exchange and format-13 telemetry |
| `ViconSource.cpp` | non-real-time Vicon acquisition and filtered Mount twist |
| `Main.cpp` | per-arm construction, planner threads, and shutdown |

## Build and hardware-free tests

```bash
cmake -S Christian_control/basic_control \
      -B Christian_control/basic_control/build
cmake --build Christian_control/basic_control/build -j2
ctest --test-dir Christian_control/basic_control/build --output-on-failure
```

The tests do not command a robot. Building the `controller` target also does not
command a robot.

## Supervised use

The supported integrated launcher is:

```bash
Christian_control/planner_bridge/scripts/run_session.sh --arm right
# or: --arm left / --arm both
```

It checks binary freshness, requires the operator to type `GO`, starts one
controller process, and waits for each selected arm's first typed Cartesian
plan activation. Requests, planner diagnostics, controller log, configuration
snapshots, and run CSVs are retained under
`runs/YYYY-MM-DD/session_HHMMSS/`. Ctrl+C or Enter stops the controller and
its in-process planner workers through the common teardown path.

`--dry-run` exercises preflight and authorization prompts without starting the
controller. A typed `GO` is only a script gate; it is not authorization to run
hardware. Robot execution still requires Christian's explicit approval for that
specific session, a clear workspace, and an immediately reachable e-stop.

Running `build/controller --arm right|left|both` directly is also robot-facing.
It starts in a zero-error Cartesian hold, captures a fixed world hold on the
first fresh Vicon sample, and waits for its in-process planner worker to
produce a typed trajectory.

## Telemetry

Format 13 records the full explanation chain: Vicon pose/twist and age,
reference identity/provenance/time, reference and measured world pose/twist,
trajectory activation/rejection/completion/cancellation/replan edges, raw task
and null-space velocities, requested/clamped/integrated joint commands, measured
joints, command acknowledgements, faults, and timing. CSV writing is buffered on
a non-critical thread; the 500 Hz loop only pushes fixed-size rows.

Use column names, not numeric positions. `scripts/runlog.py` is the schema-aware
reader.

The cyclic thread performs no terminal formatting or printing. Runtime edges
are reconstructed from the CSV; startup messages and the final decoded stop
report are printed outside normal 500 Hz operation.

## Safety

- Kortex remains in low-level servoing with actuator POSITION commands. The
  program does not use Kinova's onboard planner, obstacle avoidance, or
  self-collision avoidance.
- The sole client-side velocity limits are 76 deg/s for joints 1-4 and
  66.5 deg/s for joints 5-7. Startup refuses limits above the live hard limits.
- Bounded joints 2/4/6 use verified firmware `JOINT_LIMIT` thresholds plus a
  conservative software outward-motion boundary. Continuous joints remain
  position-unbounded.
- Every raw Cartesian-law output passes through the same velocity clamp,
  one-degree command-lead handling, position integration, and joint-boundary
  check before any Kortex send.
- A command/measured following error above 3 degrees, enabled live fault, loss
  of low-level servoing, exchange failure, stale actuator acknowledgement,
  repeated non-finite command, or repeated loop overrun stops command streaming
  through the common teardown path.
- On stop, the last position setpoint is held and SINGLE_LEVEL servoing is
  restored. Exit zero requires a clean operator stop with no observed fault.
- Offline tests prove contracts and equations, not physical calibration,
  collision clearance, human safety, Vicon latency performance, or stable
  world tracking on hardware. Those remain supervised experimental checks.
