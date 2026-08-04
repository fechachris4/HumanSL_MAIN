# Set the resolved-rate speed clip from the model limits

Date: 2026-07-22
Status: accepted; amended 2026-07-24 to remove the 0.9 factor; superseded
the uniform 45 deg/s clip ("Why the clip is 45 deg/s" in
`resolved-rate-position-integration.md`).
**Currently NOT in force — see "Reverted to 45 deg/s" below.**

## Reverted to 45 deg/s (2026-08-03)

`config::kModelVelocityLimitsDegS` — and therefore `kQdotLimitDegS` — is a
uniform **45 deg/s**, well below the model limits this record argues for.
It is a temporary bring-up value, adopted alongside the reference-source
restructure, not a repudiation of the reasoning below: everything in this
document about which limit binds low-level streaming still holds, and the
79.64/69.91 figures remain the correct Table 40 model limits.

Two consequences worth stating, because they are not obvious:

- The clip is now the *first* thing that binds, not the last. At 79.64 the
  base's own streaming enforcement was reachable; at 45 the controller
  clamps well before the base has an opinion, so the failure mode this
  record describes (setpoints outrunning an enforced limit until the
  ~5 deg ejection) is much harder to provoke.
- Anything that gates on a fraction of the clip tightens with it. The
  trajectory validator's velocity gate is 90% of the clip, so planned
  trajectories are now checked against 40.5 deg/s rather than 71.6.

Raising it back toward Table 40 is a deliberate decision, not a cleanup:
re-read this record first.

## Decision (2026-07-22, as taken)

`config::kQdotLimitDegS` is set to the Kinova general (model) limits:
79.64 deg/s for joints 1–4 and 69.91 deg/s for joints 5–7
(`config::kModelVelocityLimitsDegS`, `custom-urdf.md`). It was initially
set to 90% of those values on 2026-07-22; Christian removed that margin on
2026-07-24.

Same day, at Christian's request, the two-location design was collapsed to
one: the separate repository ceiling (`Motion.h::kCommandSpeedCeilingDegS`)
and the `static_assert` that checked the clip against it are deleted.
`kQdotLimitDegS` is a direct constexpr copy of
`kModelVelocityLimitsDegS`, so clip and model limits cannot disagree by
construction, and `Config.h` is the single place speed limits are set. Do
not reintroduce a second limit location.

## Which limit binds low-level streaming

Investigated 2026-07-22 after the former soft-limit query showed only
high-level control modes (JOYSTICK/TRAJECTORY/…) and no entry for low-level
servoing. Findings:

- The ControlConfig API has no low-level slot: every soft-limit
  getter/setter is keyed by `ControlConfig::ControlMode`, and that enum
  (ControlConfig.pb.h) contains only high-level modes — there is nothing
  to read for LOW_LEVEL_SERVOING.
- The Gen3 User Guide (R07, "Configurable limits") states: "Soft limits
  are not configurable in admittance modes nor in low-level control."
  Official position: per-mode soft limits do not govern the low-level stream.
- What provably faults in low-level is the per-actuator firmware
  safeties, Bank A (`ActuatorConfig.pb.h::SafetyIdentifierBankA`):
  FOLLOWING_ERROR, MAXIMUM_VELOCITY, JOINT_LIMIT_HIGH/LOW — the exact
  faults observed in the 2026-07-22 hardware runs. Their configured
  error/warning thresholds ARE queryable, per actuator device, via the
  DeviceConfig API (`GetAllSafetyInformation` /
  `GetSafetyConfiguration`, device ids from DeviceManager).

## Startup limit check (planned — plan item W6)

Spec:

1. Read `GetKinematicHardLimits` and fail startup if the reported hard
   limits differ from `config::kModelVelocityLimitsDegS` or if any clip
   value exceeds them.
2. Read each actuator's MAXIMUM_VELOCITY safety error threshold via
   DeviceConfig — the limit that provably binds low-level streaming —
   and fail if any clip value exceeds it.
3. On failure, print both the reported hard and actuator safety limits and
   the configured `kQdotLimitDegS` values, and name this file
   (`docs/decisions/qdot-limit-raise.md`) so the diagnosis is one read.

## Safety consequences

- Joint speeds up to ~80 deg/s are substantially faster than the previous
  45. First runs after the reconfiguration: small, nearby targets (the
  README's few-centimeter rule), workspace clear, e-stop in hand.
- ~~`Loop.cpp` currently runs a TEMPORARY experiment policy in which
  actuator/base fault bits do NOT stop the loop (marked in
  `RunResolvedRateLoop`). Restore the fault-stop exit before any
  high-speed or unattended use.~~ *(Superseded 2026-07-23: fault-stop is
  the default again — `config::kStopOnFault`, compile-time only, F2. The
  experiment is reproducible by setting it false; the Runner then warns
  loudly at startup, and observed faults still force a nonzero exit.)*
