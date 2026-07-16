# Decision: own URDF copy (GEN3_custom.urdf)

The build uses `basic_control/config/GEN3_custom.urdf`, duplicated from
`HumanSL_MAIN/config/GEN3_With_GRIPPER_DYNAMICS.urdf`, so its joint limits
are ours to edit without touching the shared model.

- Velocity limits set to Kinova general limits: 1.39 rad/s for joints 1–4,
  1.22 rad/s for joints 5–7.
- Speed validation limits for a joints-mode move are deliberately separate
  from the URDF: `kDefaultSpeedLimits` = 45 deg/s in `src/Motion.h`, checked
  against `config::kJointSpeedsDegS` at compile time (see
  `motion-txt-removal.md`). See `../known-issues.md` (50 deg/s soft limit)
  for why that cap exists.
