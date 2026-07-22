# Resolved-rate control via position integration (100 Hz, POSITION mode)

Date: 2026-07-20
Status: accepted (supersedes the VELOCITY-mode actuation of
`cartesian-velocity-controller.md`; the Cartesian DLS controller itself is
unchanged)

## Decision

Keep the Cartesian controller (Kp error → damped least squares → per-joint
clip) but actuate it by integrating the clipped joint velocities into a
persistent position command streamed in the actuators' default POSITION
control mode at 100 Hz (`src/control/Loop.cpp::RunResolvedRateLoop`):

    e = p_desired − p(q_measured);  v_d = Kp e
    q̇_raw = Jpᵀ(JpJpᵀ + λ²I₃)⁻¹ v_d
    q̇_i   = clamp(q̇_raw_i, ±45 deg/s)
    q_command += q̇_clipped · dt;   send q_command as position setpoints

No actuator control-mode switching, no settling phase — POSITION is the
default, and the takeover is the proven holding-frame pattern.

## Why VELOCITY mode was abandoned (hardware evidence, 2026-07-20)

Run log `loop_log_2026-07-20_22-32-16.csv`: the moment actuators entered
VELOCITY mode, gravity-loaded joint 2 sagged at ~5 deg/s under a commanded
zero velocity, and later moved +1.8 deg/s against a commanded −4 deg/s; the
end-effector drifted 7.7 cm from the target with the controller fighting
and losing. Joint 1 (vertical axis, gravity-free) tracked fine — in the
earlier single-joint smoke test too. Conclusion: the actuator's inner
velocity loop has no gravity compensation and insufficient stiffness
against a constant ~26 Nm load. Kinova's tracker confirms: kortex #42
(joints 2/4 drift at commanded zero, closed "not planned"), #93 (joint
speed inaccuracy), #156 (joints intermittently not responding). The
community workaround is exactly this design: compute the next position
from the desired velocity. (The run also ended on joint 6's configured
~36° position limit — arm-specific configuration, like joint 4's in
`reactive-control-removal.md` — which is independent of the actuation
mode.)

## The critical state distinction

- `q_measured`: fresh feedback each cycle; used for FK and the Jacobian
  (both from the SAME q).
- `q_command`: the persistent integrator state; what is sent.
- `q_command = q_measured` happens ONLY at startup. Resetting it from
  feedback every cycle would break the continuous integration (the
  steady-state tracking lag would be re-integrated as error). Every run
  seeds afresh — a restart cannot inherit stale integrator state or an old
  target (p_desired is re-seeded from fresh FK).

## Timing

`config::kControlDtS = 0.01` (100 Hz) is the single source of truth; the
loop grid and frequency derive from it. The integrator uses the MEASURED
elapsed cycle time, clamped to at most 2 × nominal (`ClampedCycleDt`,
`src/math/Dls.h`, unit-tested): integration stays honest under jitter, but
a scheduler stall cannot turn into one large position jump — the base's
tracking safety faults on those (`src/app/Config.h`'s clip comment).

## Why the clip is 45 deg/s, not the model limits

*(Superseded 2026-07-22: the clip is now 10% under the model limits,
conditional on reconfiguring the arm's soft limits — `qdot-limit-raise.md`.
The mechanism below still holds for whatever limit the base enforces.)*

In position streaming the base's enforced 50 deg/s soft limit empirically
causes WRONG_SERVOING_MODE faults when outrun (the joint stands still,
tracking error grows, and at ~5° the arm is kicked out of low-level
servoing — mechanism now documented at `src/app/Config.h`'s
`kQdotLimitDegS`), and the
subsystem rule caps any client-side limiting at 45 deg/s. So
`config::kQdotLimitDegS` = 45 ×7 (static-asserted against
`Motion.h::kCommandSpeedCeilingDegS`); `kModelVelocityLimitsDegS`
(79.6/69.9) remains as URDF documentation only. Per-joint clipping still
distorts the Cartesian direction when a joint saturates — accepted, as
before.

## Shutdown

Simpler than velocity mode: q_command stops updating and the position
servo holds the last setpoint — no zero-velocity frame, no control-mode
restore. Kept: stop-reason preservation, decoded post-loop report, guarded
SINGLE_LEVEL_SERVOING restore on every exit path.

## Removed

`SendVelocities`, `ActuatorConfig` usage, `LoopPhase` settling machinery
(no mode transition to settle). Git history (this commit's parent) has the
VELOCITY-mode implementation.
