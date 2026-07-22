# Raise the resolved-rate speed clip to 10% under the model limits

Date: 2026-07-22
Status: accepted — conditional on reconfiguring the arm (below); supersedes
the uniform 45 deg/s clip ("Why the clip is 45 deg/s" in
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

## Precondition — the arm must be reconfigured to match

The old 45 was 10% under the 50 deg/s speed soft limits this arm's base was
CONFIGURED to enforce. The base's limit governs regardless of the client:
position setpoints stepping faster than it are not followed, and once the
tracking error reaches ~5 deg the arm faults out of low-level servoing
(WRONG_SERVOING_MODE — mechanism in `Motion.h`'s ceiling comment).

Before the next hardware session:

1. Raise the arm's joint speed soft limits to the model limits
   (79.6/69.9 deg/s) in the Kinova web dashboard (`http://192.168.1.10`).
2. Verify with `tools`' `./query_limits` that the base reports the raised
   soft limits.

If the base still enforces 50 deg/s, any motion demanding more than
~50 deg/s on a joint faults mid-move — the exact failure the old ceiling
existed to prevent.

## Safety consequences

- Joint speeds up to ~72 deg/s are substantially faster than the previous
  45. First runs after the reconfiguration: small, nearby targets (the
  README's few-centimeter rule), workspace clear, e-stop in hand.
- `Loop.cpp` currently runs a TEMPORARY experiment policy in which
  actuator/base fault bits do NOT stop the loop (marked in
  `RunResolvedRateLoop`). Restore the fault-stop exit before any
  high-speed or unattended use.
