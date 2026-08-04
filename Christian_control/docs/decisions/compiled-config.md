# Config.h is the whole configuration, and what it no longer explains

Date: 2026-08-03
Status: accepted

## Decision

`basic_control/src/Config.h` holds every setting the controller has, as
compiled constants. `--log` is the only runtime argument. Changing
behaviour means editing that file and rebuilding.

This record exists because Config.h was reduced to one line per constant on
2026-08-03. The values and their warnings stayed in the header; the
rationale — incident narratives, hardware findings, sizing evidence — moved
here and into the records named below. Nothing was deleted outright.

## Why there is no runtime configuration

The CLI/TOML front-end (`Options.{h,cpp}`, `config/control.toml`, `--kp`,
`--config`) was removed on 2026-08-03; `runtime-config.md` records what it
did and why it existed. What replaced it is nothing: with one operator, one
arm, and a rebuild that takes seconds, a second configuration surface was
somewhere for the compiled defaults and the effective values to disagree.

The consequence is a real one and should not be softened: **the compiled
constants are the only thing between the operator and the arm.** Reading
Config.h before a session is not optional, because there is no flag that
can override a bad value at the last moment.

Safety policy was never runtime-settable even when the front-end existed
(`kStopOnFault` is compile-time by deliberate design), so that part is
unchanged.

## Configured joint limits do not survive a power cycle

Found 2026-08-03, and the reason `Connect::EnsureJointLimits` re-applies
the bounded-joint 2 / 4 / 6 `JOINT_LIMIT` thresholds on **every** connection
rather than once.

- Values written through `DeviceConfig::SetSafetyConfiguration` were
  written and read back correctly, then found back at 0/0 about an hour
  later, after a power cycle.
- Kortex 2.7.0 exposes no save/commit RPC — there is no API call that makes
  the write durable.
- A later live read found joint 2's HIGH/LOW warning and error thresholds
  back at 0/0. The earlier persistence observation was stale; joint 2 now
  receives the same per-connection read/correct/readback treatment.

Re-applying per connection is therefore a stopgap, not the solution. What
it buys is that the arm can never run with a degenerate empty band: joints
2, 4, and 6 are bounded, and a 0/0 threshold band faults outward motion.

The written magnitudes are j2 ±130 warn / ±140 error, j4 ±145 / ±150, and
j6 ±118 / ±123. The software limits stay primary and the firmware is a
backstop.

Joints 1/3/5/7 are left at 0 — they are continuous, so no position limit
applies.

## What stopped being a stop (2026-08-03)

Three guards were removed the same day. Each is now telemetry: the evidence
is still recorded every cycle, but none of it ends a run.

- The **0.5 s takeover-hold window** was trimmed. Control starts directly
  from one measured-position seed frame; the loop's guards cover the same
  ground from the first cycle.
- The **short-window motion-response timeout** and the
  **unchanged-acknowledgement stop** became `req_j*` / `cmd_j*` / `meas_j*`,
  `lead_limited_j*` and `ack_unchanged_j*` columns.

**Consequence, stated plainly: a completely unresponsive arm no longer
stops the program by itself.** The already-clamped joint velocity first
forms a proposed command; the lead projection targets a 1 deg gap from the
wrapped measurement. A final envelope then limits the sent command delta to
`abs(qdot_clamped * dt)`. On discontinuous feedback that envelope wins, so
lead recovery can temporarily remain above 1 deg and the unchanged 3 deg
following-error stop is reachable as the backstop. For a frozen plant the
backstops remain the operator and the robot's own firmware limits. Attended
use only.

The saturation stop went earlier, on 2026-07-23, for a different reason: a
pinned velocity clamp is normal transit toward a far target, not a fault.
The clamp itself still limits speed.

## Where the rest went

- Velocity clip, and why it is 45 rather than 79.64/69.91 —
  `qdot-limit-raise.md`.
- Startup fault clearing, reversed from the 2026-07-20 decision —
  `fault-handling-hardening.md`.
- Reactive-pose gains and the staged term switches —
  `reactive-pose-port.md`.
- Why there is no Cartesian velocity/acceleration/workspace limiting —
  `cartesian-velocity-controller.md`.
- Following-error limit evidence —
  `resolved-rate-position-integration.md`.
