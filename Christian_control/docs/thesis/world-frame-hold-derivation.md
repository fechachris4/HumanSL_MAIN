# World-frame hold — derivation and code map

Written 2026-08-13. Working mode: equations before code — every control
change is presented here first, then implemented. Each equation is
mapped to the code that realises it (or will), in both projects:

- sim: `~/msc_project/controller/` (Python, MuJoCo) — the reference
  architecture, per Christian's decision of 2026-08-13.
- hardware: `Christian_control/control/` (C++).

## 1. Frames and symbols

| Symbol | Frame / quantity | Where it comes from |
|---|---|---|
| `W` | world (Vicon origin) | Vicon DataStream, `192.168.128.206:801` |
| `T` | torso — a tracked body of its own | Vicon segment (does not exist yet in Nexus; targets will be specified relative to it) |
| `MS` | mount segment — the marker cluster on the backpack plate | Vicon `Mount` segment |
| `B` | an arm's `base_link` | fixed offset from `MS` (hardware) / from `T` (sim's `MountCalibration`) |
| `E` | end-effector | FK of the arm chain |
| `q, q̇` | joint positions/velocities (7) | actuators |
| `J_B` | 6×7 Jacobian in base axes | `Kinematics.h` (Pinocchio), sim: `pin.getFrameJacobian(..., LOCAL_WORLD_ALIGNED)` |
| `world_T_x` | pose of frame x in world | homogeneous transform |

The hardware chain and the sim chain differ by one link:

```
sim:       W ← T (plant, perfect)      ← B (MountCalibration, fixed) ← E (FK)
hardware:  W ← MS (Vicon, 100 Hz ZOH) ← B (MS_T_B, to be calibrated) ← E (FK)
```

The sim's `MountCalibration` (torso→base, `frames.py:133`) plays the
role hardware calls `mountseg_T_mount` composed with the plate→base_link
geometry. One fixed transform either way.

## 2. The law that exists (identical in both projects)

`ReactiveLaw.h` was ported from `reactive_controller.py` and
cross-validated on identical inputs. Its equations, numbered as in the
header:

```
(1)  e_p = p_d − p_E              e_R = log3(R_d · R_Eᵀ)
(2)  e_v = v_ref − (J q̇)_lin      e_ω = ω_ref − (J q̇)_ang
(3)  V_cmd = [ K_p,p e_p + K_d,p e_v ]
             [ K_p,R e_R + K_d,R e_ω ]
(4)  q̇_task = Jᵀ (J Jᵀ + λ² I₆)⁻¹ V_cmd          (damped least squares)
(5)  deadband joint-limit objective q̇_null
(6)  q̇_raw = q̇_task + (I₇ − Jᵀ(JJᵀ+λ²I)⁻¹J) q̇_null
```

Code: `RotationLog` (ReactiveLaw.h:66), `TwistError` (:79), `TaskTwist`
(:91), `DampedLeastSquares6` (:112), `LimitAvoidanceVelocity`;
sim: `pose_error` (reactive_controller.py:118), `twist_error` (:130),
`task_twist_terms` (:142).

**The law is frame-agnostic.** Nothing in equations (1)–(6) names a
frame. The controller's meaning is decided entirely by which frame the
inputs `p_E, R_E, J, p_d, R_d` are expressed in. Today the hardware
feeds base-frame inputs, so the arm holds relative to the backpack. The
sim feeds world-frame inputs, so it holds in the world. That input
assembly is the whole difference — this is why "establish world on
hardware" is an adapter, not a new controller.

## 3. World-state assembly — the sim's, and the hardware target (Architecture A)

Chosen 2026-08-13: hardware mirrors the sim's assembly
(`frames.py:120`, `arm_controller_state`). Per 500 Hz cycle:

```
(A1)  world_T_B(t) = ZOH[world_T_MS](t) · MS_T_B         # Vicon sample held
(A2)  p_E^W = world_T_B · FK(q)          R_E^W = R_WB · R_E^B
(A3)  J^W   = diag(R_WB, R_WB) · J_B                      # frames.py:147-150
(A4)  e_p = p_d^W − p_E^W                e_R = log3(R_d^W · (R_E^W)ᵀ)
(A5)  law (1)–(6) unchanged, with J := J^W
```

Sim lines: (A1)=`frames.py:130-135` (T_W_T @ T_T_B), (A2)=`:138-139`,
(A3)=`:147-150`, then the unchanged law. Hardware: these lines do not
exist yet; they will live in the state-assembly step of
`Controller.cpp`, with `world_T_MS` arriving from the acquisition slot
(section 5).

Architecture B (world target re-expressed in base each cycle,
`p_ref^B = T_B_W p_d^W`, law fed base-frame inputs) is mathematically
the same error rotated: `e^B = R_BW e^W`. It was considered and not
chosen; A mirrors the sim exactly and needs no rework when twist
handling arrives.

## 4. The base-motion term — derived now, used later

Differentiate `p_E^W = p_B^W + R_WB p_{E/B}^B`:

```
(B1)  v_E^W = v_B^W + ω_B^W × (p_E^W − p_B^W) + (J^W q̇)_lin
(B2)  ω_E^W = ω_B^W + (J^W q̇)_ang
```

The base's contribution, transported to the end-effector point:

```
(B3)  V_base,E = [ v_B + ω_B × (p_E − p_B) ]
                 [ ω_B                      ]      so   V_E^W = V_base,E + J^W q̇
```

**Already implemented in the sim**: `frames.py:154-161` builds
`ee_twist_world` exactly as (B1)–(B2); `twist_error`
(reactive_controller.py:130) then subtracts it, so the sim's Kd term
damps the *world* twist and a moving torso feeds forward into the
command. The commanded-joint-velocity form is:

```
(B4)  J^W q̇ = V_cmd − V_base,E
```

**Deliberately absent from hardware slices 1–2** (Christian, 2026-08-13:
"establish world on hardware using the vicon. which uses only
feedback"). With `V_base,E := 0`:

- rejection of base motion happens only through (A4): base motion moves
  `p_E^W`, the error grows, `K_p` pulls it back;
- against a steady base velocity `v_B`, the standing world error is

```
(B5)  e_p,ss ≈ v_B / K_p,p  +  v_B · t_age
```

  (first-order lag of a P loop tracking a ramp, plus the staleness
  term). At `K_p = 2 s⁻¹`, `v_B = 0.1 m/s`, `t_age ≈ 15 ms`:
  ≈ 51.5 mm. This number is the measurable cost of feedback-only, and
  the measurable benefit of enabling (B4) later — the ablation the
  evaluation chapter runs;
- the Kd term still damps `J q̇` (mount-relative), not the world twist —
  stabilising, but frame-inconsistent with (B1); consistency arrives
  with the same input `V_base,E` needs. Both change together in a later
  slice.

## 5. The sample contract (hardware only; the sim never needed one)

The sim reads perfect torso state every cycle. Hardware reads Vicon at
~100 Hz into a 500 Hz loop: one sample serves ~5 cycles. Decided
2026-08-13 (strict minimum for the first hardware run):

```
BasePoseSample:
  frame_number       # Vicon's own counter
  sequence           # ours; increments only on a NEW sample
  t_receive_s        # steady_clock at SDK read
  latency_reported_s # SDK GetLatencyTotal
  world_T_MS         # the pose (mm→m applied exactly once)
  valid              # occlusion / quaternion sanity
age(t) = t_now − t_receive_s     # computed and logged every cycle
```

Cycle rules:

- `sequence` unchanged ⇒ reuse the pose (zero-order hold); the age
  column grows. **Never finite-difference across a reused sample** — at
  100 Hz→500 Hz a naive derivative is zero four cycles then a spike.
  Slice 1–2 avoid this trivially: nothing differentiates Vicon data.
- `age > threshold` or `!valid` ⇒ **freeze the world correction at the
  last good value**; the law keeps running (base-relative behaviour),
  freshness is logged as a graded measure. No stop, no step. Christian's
  rule, verbatim: "A controller that uses stale Vicon data without
  knowing it is stale is more dangerous than one that has no Vicon
  integration."

Deferred, with the condition that brings each in: velocity estimation +
filtering (with `V_base,E`); interpolation/extrapolation of the base
pose (only if slice-2 logs show the ZOH staircase measurably limits the
hold); torso-frame targets (needs the Torso segment in Nexus).

## 6. Slices

**Slice 1 — see the world (no control change).** Acquisition thread
(owns the SDK connection) → single-writer slot → the 500 Hz loop samples
without blocking → log columns (`world_T_<segment>` for all five
segments, plus the contract fields and age) → panel columns appear via
the name-based reader. Exit criteria before slice 2: age histogram over
a real session is sane; occlusion behaviour observed and logged;
signs/frames verified against hand-measured geometry.

**Slice 2 — hold in the world (control change, own design gate).**
Assembly (A1)–(A5) behind a config switch, staged bring-up, the
`PoseReference` producer anchoring `p_d^W` at activation. Gate:
design review + one adversarial review (motion-path code) + supervised
hardware session, per CLAUDE.md.

## 7. Evaluation (a deliverable, not a by-product)

Christian, 2026-08-13: wiring proves integration; it does not prove
stabilisation. Metrics logged from slice 1 onward:

- world-frame hold error `‖e_p^W‖` (RMS, max) versus base-motion
  magnitude, from the controller's own log;
- sample age distribution; occlusion count and durations per session;
- later ablations on the same rig: feedback-only vs `V_base,E` enabled
  ((B5) predicts the difference); ZOH vs extrapolation if ever built.

## 8. What could make this document wrong

- The axis convention and segment template origins are still unread
  from Nexus — a sign error there invalidates (A1) silently. Read them
  before slice 2.
- `MS_T_B` (marker plate → base_link) is uncalibrated. For a *hold*,
  a constant error here largely cancels in (A4) (both `p_d^W` anchored
  and `p_E^W` measured carry it); it degrades with base rotation
  magnitude. Slice 2 can start crude; calibration sharpens it.
- `Mount22` marker naming and the missing Torso segment are open Nexus
  items on the frame-contract record.
