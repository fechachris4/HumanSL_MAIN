# Config.h is the whole configuration, and what it no longer explains

Date: 2026-08-03
Status: accepted

## Decision

`control/Config.h` holds every setting the controller has, as
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

Firmware `JOINT_LIMIT` is the actual joint-position enforcement; the
controller has no separate client-side joint-position clamp. Instead, its
software warning guard holds the last safe full command frame and stops
before a bounded joint moves farther outward past the warning, while still
allowing inward recovery. The velocity clip, spherical input reach screen,
and following-error stop are separate protections. The written warnings are
j2's established ±130, j4 ±145
(inside its documented ±147.8 range), and j6 ±118 (inside ±120.3). The
errors ±140 / ±150 / ±123 lie outside the documented respective
±128.9 / ±147.8 / ±120.3 ranges.

Joints 1/3/5/7 are left at 0 — they are continuous, so no position limit
applies.

## Takeover hold and acknowledgement safety (2026-08-04)

Before any controller or integrator state can produce a target-directed
setpoint, the low-level loop streams the exact Seed joint positions for
0.5 s (250 frames at 500 Hz). Every hold reply is logged and applies the
same following-error, low-level-servoing, live-fault, and stale-
acknowledgement precedence as normal control; a failed hold restores
SINGLE_LEVEL without beginning controller motion. The final hold reply is
the only measurement used to seed the controller and integrator.

The short-window physical motion-response timeout remains absent: cyclic
freshness demonstrates downstream acknowledgement progress, not physical
motion or setpoint acceptance. An acknowledgement that stays unchanged for
the compiled threshold does stop the run, and the `req_j*` / `cmd_j*` /
`meas_j*`, `lead_limited_j*`, and `ack_unchanged_j*` columns retain the
per-cycle evidence.

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
