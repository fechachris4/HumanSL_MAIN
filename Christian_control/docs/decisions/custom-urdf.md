# Decision: own URDF copy (GEN3_custom.urdf)

The build uses `basic_control/config/GEN3_custom.urdf`, duplicated from
`HumanSL_MAIN/config/GEN3_With_GRIPPER_DYNAMICS.urdf`, so its joint limits
are ours to edit without touching the shared model.

- Velocity limits set to Kinova general limits: 1.3893 rad/s for joints 1–4,
  1.2200 rad/s for joints 5–7 (79.6 / 69.9 deg/s; authoritative deg/s array:
  `config::kModelVelocityLimitsDegS` in `src/app/Config.h`, updated
  2026-07-20 — keep it and the URDF literals in sync by hand).
- Speed validation limits for a joints-mode move are deliberately separate
  from the URDF: `kDefaultSpeedLimits` = 45 deg/s in `src/Motion.h`, checked
  against `config::kJointSpeedsDegS` at compile time (see
  `motion-txt-removal.md`). The cap existed because the arm's base was
  configured to enforce a 50 deg/s joint speed soft limit that faults
  position streams outrunning it (mechanism now documented at
  `src/app/Config.h`'s `kQdotLimitDegS`; raised 2026-07-22 —
  `qdot-limit-raise.md`).
