# Milestone 2 — MPC + on/off toggle (design record)

**Date:** 2026-06-10 · **Status:** Implemented, tests green (pending independent review)

## Goal

Add a real MPC behind the M1 `JointController` seam, and a runtime **toggle** so the
operator can switch the arm between a simple controller (off) and the MPC (on).
Laptop-testable, no robot, no new dependencies.

## Key decision: no external QP solver

The MPC is an **Eigen-only condensed quadratic program** — no OSQP/qpOASES. Adding a
bundled solver to this repo without hardware is a yak-shave; a box-unconstrained
quadratic MPC with limits enforced by clamping needs only a linear solve. OSQP (for
*hard* in-QP inequality constraints) remains a documented future upgrade.

## Formulation (per joint, decoupled)

Model: double integrator on an internal **reference** state `x = [q_ref, v_ref]`,
acceleration input `u`, `A=[[1,dt],[0,1]]`, `B=[[0.5dt²],[dt]]`.

Cost over horizon `N`:
```
J = Σ_{k=1..N-1} w_q (q_k-goal)² + w_qf (q_N-goal)²    (position tracking + terminal)
  + Σ_{k=1..N-1} w_v v_k²        + w_vf v_N²            (velocity damping + arrive-at-rest)
  + Σ_{k=0..N-1} w_u u_k²                               (effort)
```
Condensed (states eliminated): `q_k = Sq[r]·x0 + Tq[r,:]·U`, `v_k = Sv[r]·x0 + Tv[r,:]·U`
with `Tq(r,j)=dt²(0.5+(r−j))`, `Tv(r,j)=dt`, `Sq(r)=[1,(r+1)dt]`, `Sv(r)=[0,1]`.
Unconstrained optimum solves the SPD system
```
(Tqᵀ Wq Tq + Tvᵀ Wv Tv + w_u I) U = goal·(Aq·1) − (Aq Sq + Av Sv)·x0,   Aq=TqᵀWq, Av=TvᵀWv
```
Matrices are constant (model + weights), so `H` is **LLT-factorized once** and reused for
all 7 joints every tick. Apply the first input `u0` via **semi-implicit Euler** with
velocity then position clamps (anti-windup); the clamped reference is the emitted setpoint.

`[goal, 0]` is a zero-input fixed point (proven: at `x0=[goal,0]`, the linear term is
`goal·Aq·1 − Aq·Sq·[goal;0] = 0` since `Sq·[goal;0]=goal·1`), so there is **no
steady-state offset**.

## The overshoot bug (and fix)

First implementation had position + effort cost only → **underdamped**: the reference
overshot the goal by ~15% and oscillated, so closed-loop convergence failed. Adding
velocity cost (`w_v` running, `w_vf` terminal) damps it. A sweep of `w_v` (open-loop,
0.3 rad step) found:

| w_v | overshoot | settle |
|-----|-----------|--------|
| 0.0 | 0.044 | 4.7 s |
| 0.05 | 0.012 | 3.7 s |
| **0.1** | **0.0004** | **2.9 s** |
| 0.2 | 0 | 5.7 s |
| 0.5 | 0 | 8.0 s |

`w_v=0.1` is the knee — ~critically damped, fastest settle. Adopted as default.

## Toggle — `SwitchableController`

Owns the simple controller and the MPC; delegates `compute()` to the active one.
`set_use_mpc(on, state)` flips the toggle and **resets the newly-active controller to
the measured state** (bumpless transfer — no setpoint jump). The driver is unchanged
(it holds one `JointController`). This is what a "MPC on/off" voice/CLI command drives.

`JointController` gained `reset(state)` (default no-op; overridden by `RateLimited` and
`Mpc`). `ControlDriver::step()` now returns the applied setpoint and `run()` loops over
it (removed the M1 duplication flagged in review).

## Tests (all green, laptop, no robot)

MPC: converges end-to-end; reference respects the velocity limit; cost weights change
behavior (higher `w_u` → slower); accelerates gradually (peak velocity not at t=0).
Toggle: starts simple; reports MPC after switch; changes output; bumpless (no jump).
Plus `HoldController` coverage (gap flagged in the M1 review). Total 22/22.

## Honest limitations (carried into M3)

- **Feedforward only.** The MPC integrates its *own* reference and ignores measured
  state except on reset → no disturbance rejection / no feedback MPC. Convergence relies
  on the plant tracking the reference. Real closed-loop MPC (using measured `q,dq` with
  the true dynamics model) is the upgrade.
- **Decoupled per-joint double integrator** — no inertia/coupling/gravity. That is
  exactly what the **M3 torque path** introduces.
- **Limits via clamp, not hard QP constraints** — OSQP upgrade.
- Still no integration with the real `TrajectoryRealTime` execution thread (M3 adapter).
