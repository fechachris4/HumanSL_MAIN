# Control-loop and hardware-safety standard

This standard applies to robot control, Kortex and Vicon boundaries, feedback,
timing, safety monitoring, and hardware-facing tests. Subsystem instructions may
add stricter constraints but may not weaken these rules.

## Hardware boundary

- Isolate Kortex, Vicon, filesystem, and clock APIs behind small boundary
  components. Keep controller mathematics hardware-free and deterministic.
- Establish sessions, load models/configuration, allocate buffers, and validate
  limits before entering a critical loop.
- Contain SDK exceptions at the narrow hardware exchange boundary and translate
  them immediately into explicit status. Do not use exceptions as loop control.
- Check every result that affects command validity, robot state, servoing mode, or
  fault handling. An unchecked SDK response is a defect.
- Treat any new hardware command path as safety-critical. Document its limits,
  startup behavior, stop behavior, and operator prerequisites before execution.

## State and timing

- Perform at most one robot-state exchange per control tick. Create one immutable
  state snapshot and pass it through estimation, safety, control, and logging.
- When a command exchange also returns feedback, reuse that feedback rather than
  issuing a second read.
- A snapshot records a monotonic timestamp or sequence, validity/status, and the
  state needed by the controller. Reject stale, repeated, non-finite, or
  dimensionally invalid state before calculating a new command.
- Use `std::chrono::steady_clock` for durations and loop scheduling. Pace fixed-rate
  loops with `sleep_until` on a fixed time grid; do not drift with `sleep_for`.
- Define the missed-deadline policy for every hardware loop. Do not silently
  continue indefinitely after repeated overruns or stale feedback.

## Moderate real-time policy

Inside the critical loop:

- do not allocate or free memory;
- do not resize containers or Eigen objects;
- do not perform file I/O or terminal output;
- do not deliberately throw, and do not allow exceptions to escape the hardware
  exchange boundary;
- do not acquire locks with unbounded wait time;
- do not parse configuration, create clients, reconnect, or load models;
- do not call code whose timing or allocation behavior has not been audited.

Preallocate workspaces, state snapshots, commands, and telemetry buffers. A
simulation or offline loop may relax these constraints only when it is clearly
separate from the hardware-critical implementation.

## Command validation and safe stop

- Before transmission, verify command dimensions, finite values, units, frames,
  joint/actuator limits, rate limits, and any controller-specific invariants.
- Clamp only where clamping is part of the documented safety policy. Never use
  silent clamping to conceal an invalid controller output.
- Hardware-critical failures include stale or invalid feedback, SDK exchange
  failure, robot/actuator fault, violated tracking or limit invariant, repeated
  deadline miss, and non-finite controller output.
- On a hardware-critical failure, stop generating new motion immediately and
  enter the subsystem's documented safe-stop path. Preserve the first failure
  reason and do not overwrite it during cleanup.
- The exact safe-stop action is subsystem-specific and must be documented and
  tested. It may include a bounded hold at measured position, stopping the stream,
  an SDK-supported stop, and restoration of servoing mode. Do not invent gripper
  or retreat motion as a generic response.
- Cleanup and destructors must not throw. Servoing/session restoration must be
  attempted on every exit path where communication remains valid.

## Logging and controller placement

- Emit structured, low-rate events for mode changes, warnings, faults, safe stops,
  configuration, and experiment lifecycle.
- Send high-frequency telemetry to a preallocated buffer and write CSV or binary
  data from a non-critical context. Never format or flush each sample in the
  critical loop.
- Record commanded and measured values with timestamps, units, frames, status,
  and fault information sufficient to reconstruct a failure.
- Controller equations and safety decisions belong in explicit controller/safety
  components, not in logging, SDK adapters, callbacks, or `main.cpp`.
- Never execute a hardware-facing binary as an incidental build, sanitizer, or
  smoke test. Follow the authorization rules in the root and nested `AGENTS.md`.
