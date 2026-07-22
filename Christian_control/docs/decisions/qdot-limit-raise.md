# Raise the resolved-rate speed clip to 10% under the model limits

Date: 2026-07-22
Status: accepted; supersedes the uniform 45 deg/s clip
("Why the clip is 45 deg/s" in
`resolved-rate-position-integration.md`)

## Decision

`config::kQdotLimitDegS` is raised from a uniform 45 deg/s to per-joint
values 10% under the Kinova general (model) limits: ≈71.6 deg/s for joints
1–4 and ≈62.9 deg/s for joints 5–7 (model limits 79.6/69.9 —
`config::kModelVelocityLimitsDegS`, `custom-urdf.md`).

Same day, at Christian's request, the two-location design was collapsed to
one: the separate repository ceiling (`Motion.h::kCommandSpeedCeilingDegS`)
and the `static_assert` that checked the clip against it are deleted.
`kQdotLimitDegS` is now *derived* at compile time in `Config.h` as
`kQdotLimitSafetyFactor` (0.9) × `kModelVelocityLimitsDegS` — a constexpr
computation, so clip and model limits cannot disagree by construction, and
`Config.h` is the single place speed limits are set. Do not reintroduce a
second limit location.

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
   limits differ from `config::kModelVelocityLimitsDegS` (the clip
   derivation assumes them) or if any derived clip value exceeds them.
2. Read each actuator's MAXIMUM_VELOCITY safety error threshold via
   DeviceConfig — the limit that provably binds low-level streaming —
   and fail if any clip value is at or above it.
3. On failure, print both the reported hard and actuator safety limits and
   the derived `kQdotLimitDegS` values, and name this file
   (`docs/decisions/qdot-limit-raise.md`) so the diagnosis is one read.

## Safety consequences

- Joint speeds up to ~72 deg/s are substantially faster than the previous
  45. First runs after the reconfiguration: small, nearby targets (the
  README's few-centimeter rule), workspace clear, e-stop in hand.
- `Loop.cpp` currently runs a TEMPORARY experiment policy in which
  actuator/base fault bits do NOT stop the loop (marked in
  `RunResolvedRateLoop`). Restore the fault-stop exit before any
  high-speed or unattended use.
