# World-hold slice — design (slice 2 of the world-frame work)

Date: 2026-08-13. Status: awaiting Christian's approval; no code yet.
Parent derivation: `Christian_control/docs/thesis/world-frame-hold-derivation.md`
(equations (A1)–(A5), (B5), §5 sample contract, §6 slices).
Decisions baked in (Christian, this date, interactive): **auto-engage when
fresh**, **50 ms freeze + re-anchor on recovery**, **no Nexus-reads gate —
the first tethered run verifies signs itself**.

## 1. The behaviour, in equations first

Anchor, at the engage instant `t0` (first cycle where the Vicon sample is
fresh and the needed segments valid):

```
world_T_B(t)  = world_T_MS(t) · MS_T_B                    (A1, ZOH sample)
X_des^W       = world_T_B(t0) · FK(q(t0))                  (anchor once)
```

`MS_T_B` in this slice: `MSeg_T_mount` is ASSUMED IDENTITY (the Nexus
template origin taken as the URDF `mount` frame — unverified until the
GUI reads); `mount_T_base_link` comes from the Pinocchio model, computed
once at startup. The §8 argument of the derivation doc applies: a hold
mostly cancels a constant error here; the residual scales with base
rotation, and the first run is the sign check.

Per cycle while engaged (500 Hz):

```
ref^B(t)      = T_B_W(t) · X_des^W                         (the seam image)
e_p^W         = X_des^W.p − [world_T_B(t)·FK(q)].p         (logged, watched)
```

**Revised 2026-08-13 evening (Christian's direction): the simulation is
the reference implementation — map it, don't re-derive.** The earlier
paragraph here proposed handing the law a base-frame image through the
existing seam; that was an unjustified deviation from the sim convention
and is withdrawn. The hardware mirrors `msc_project`'s cycle exactly:

```
sim (runner.py:138 cycle)                 hardware (per 500 Hz cycle)
─────────────────────────                 ──────────────────────────
read_state → PlantState                   feedback q,q̇ + BasePoseSlot ZOH
resolve_targets_world (per cycle)         Frames::ResolveTargetWorld
controller_states (world assembly)        Frames::ArmControllerState
solve_reactive_velocity (world in)        ReactiveLaw.h fed WORLD inputs
constrain/limits → actuate                existing clip/limits/integration
```

The law's Cartesian inputs become world-frame at the call site in
`Controller.cpp` (errors in world, `J^W = diag(R_WB,R_WB)·J_B` exactly as
`frames.py:147-150`), which is what Architecture A meant all along. The
`PoseReference` seam — currently unwired in production, so this is not a
behaviour change — adopts the sim's `WorldTarget` semantics: pose and
feed-forward twist in the WORLD frame, resolved per cycle. Existing
base-frame configured targets become framed targets resolved to world
each cycle (sim `TargetFrame.BASE` semantics — identical behaviour to
today, including under base motion, since a base-framed target moves
with the base). When `V_base,E` arrives it enters exactly as
`frames.py:154-161` builds `ee_twist_world` — zero rework.

Ramp-in (new, risk-shaping for auto-engage): for `kWorldHoldRampS` after
every (re-)engage, the reference is interpolated from the engage-time
pose toward `ref^B(t)`:

```
ref_used^B = FK(q(t0'))·(1−r) ⊕ ref^B(t)·r,   r = UnitRamp(t−t0', kWorldHoldRampS)
```

(⊕: position lerp + rotation slerp; `t0'` = this engage's instant.) A
wrong sign therefore appears as a slow, visible drift the operator kills,
not a step at full gain. `UnitRamp` is the pattern the law already uses
for limit-avoidance.

**Implementation corrections (2026-08-13, during build):** (1) the ramp
interpolates from the CURRENT pose toward the anchor —
`target = (1−r)·x_now ⊕ r·X_des` — so the effective error is `r·e`; the
formula above interpolated between two constants that coincide at engage
and would have ramped nothing. (2) The divergence latch watches BOTH
error channels: position (`kWorldHoldMaxErrorM`, 0.08 m) and rotation
(`kWorldHoldMaxRotErrorRad`, 0.5 rad), because a wrong rotation sign can
reorient the arm while position error stays small. (3) No
`WorldHoldSource` in Targets was needed: the controller already owned
the hold pose (`TrackingController::Reset`), so `WorldHold` lives inside
the controller's hold path — trajectory precedence is the existing joint
branch (which now drops the anchor), and an explicit pose reference
outranks the hold. Fewer moving parts than §3's file list assumed.

## 2. State machine

```
        takeover hold (unchanged, today's path)
              │ auto: sample fresh (age < kWorldHoldFreshMaxAgeS)
              │       AND Mount segment valid          [checked each cycle]
              ▼
   ┌─── WORLD_HOLD (anchored, ramping in) ──────────────┐
   │  fresh: ref^B = T_B_W·X_des^W                      │
   │  stale/invalid > 50 ms: FREEZE ref^B at last good  │
   │      (= today's base-frame hold; logged, no stop)  │
   │  recovered: RE-ANCHOR X_des^W at current pose,     │
   │      ramp in again (hold_reanchor_count++)         │
   └── divergence: ‖e_p^W‖ > kWorldHoldMaxErrorM ───────┘
              │
              ▼
   joint hold + WORLD-HOLD LATCHED OFF for this run
   (printed + logged; prevents an engage/diverge/engage
    oscillation when the sign is wrong — degradation to
    today's behaviour, never a stop)
```

A trajectory arriving on the stdin pipe disengages the world hold in
favour of the trajectory (existing behaviour wins); after the trajectory
completes, re-engage happens through the same fresh+valid check and ramp.

## 3. Simulation component → hardware equivalent → genuinely new logic

| sim (source of truth) | hardware equivalent | genuinely new (hardware-only) |
|---|---|---|
| `world.read_state`: torso pose from MuJoCo ground truth, perfect every step (`sim/world.py:read_state`) | `world_T_MS` from `BasePoseSlot` (slice-1 code, validated live) | 100→500 Hz ZOH; freshness gate; never-seen ⇒ `world_T_B = I` so world≡base and behaviour is exactly today's |
| `MountCalibration` `T_T_B`, fixed, read from the model file (`sim/world.py:71`, `_mount_pose`) | `MS_T_B = MSeg_T_mount · mount_T_base_link`; the second factor read once from the Pinocchio model at startup | `MSeg_T_mount` assumed identity until stage-2 calibration — the assumption the first tethered run tests |
| `frames.arm_controller_state`: `T_W_B = T_W_T·T_T_B`; `ee_pose_world`; `J^W` = R_WB-rotated LOCAL_WORLD_ALIGNED Jacobian (`frames.py:120-150`) | new `src/Frames.h/.cpp`, mirroring frames.py function-for-function (the C++-mirrors-Python rule) | — (same maths, same names) |
| `ee_twist_world` includes torso-twist transport (`frames.py:154-161`) | `J^W q̇` only — base twist treated as zero | **called-out difference**: no measured base twist exists yet; his feedback-only decision; cost quantified by (B5). Removed when `V_base,E` lands |
| `resolve_target_world`: framed target → world ONCE PER CYCLE, twist transported (`frames.py:223`) | `Frames::ResolveTargetWorld`, same semantics; existing fixed targets = `TargetFrame::BASE`; world hold = `TargetFrame::WORLD` anchored at engage | anchor-at-current-pose as the world target's origin; arbitration: joint trajectory wins while active |
| `pose_error`/`twist_error`/`solve_reactive_velocity` on world quantities (`reactive_controller.py:118-212`) | `ReactiveLaw.h` unchanged, fed world inputs at the `Controller.cpp` call site | — (already cross-validated against the sim law on identical inputs) |
| undamped `pinv` null projector | damped projector | pre-existing documented deviation (ReactiveLaw.h header), hardware-motivated; keep |
| continuous MuJoCo `q` | wrap-to-(−π,π] deadband | pre-existing documented deviation (Kortex reports [0,360)); keep |
| `constrain_velocity_for_human` (OSQP CBF filter) | existing velocity clip + joint limits + guards | filter port is future work (recorded direction), not this slice |
| — (sim state is always perfect) | — | stale>50 ms freeze; re-anchor on recovery; 2 s ramp-in; divergence latch >8 cm → joint hold. All of it is the Vicon trust boundary the sim never needed |

Files: new `src/Frames.h/.cpp` + `src/WorldHold.h` (pure state machine) +
`tests/test_frames.cpp` (cross-checked against `frames.py` fixtures, the
same technique test_reactive_law already uses) + `tests/test_world_hold.cpp`;
edits to `Targets` (WorldHold source + framed resolution), `Controller.cpp`
(world call site), `Config.h` (four constants appended; the uncommitted
velocity edit untouched), `Runner` (hand the slot sample to the source),
`Hardware.*` (log_format 11: `hold_state`, `world_err_m`,
`world_err_rot_rad`, `hold_ramp`, `hold_reanchor_count`).

## 4. Offline acceptance (before any hardware)

- All existing tests unchanged and green (no behaviour change while the
  hold is disengaged or latched).
- WorldHold unit tests cover: engage gating, freeze at exactly the
  threshold, re-anchor producing zero instantaneous error, ramp
  continuity (no reference step at engage/freeze/recover boundaries),
  divergence latch one-way, trajectory precedence.
- The A↔seam identity test passes at 1e-12 on randomized configurations.

## 5. Supervised bring-up (Christian present, e-stop in hand)

Per his decision the first run IS the sign check: tethered, low Cartesian
gains (panel: drop `kp`/`kp_rot` to ~25% first), workspace clear, watch
`world_err_m` on engage — shrinking ⇒ signs right, growing ⇒ ramp +
divergence latch demote it to joint hold and the run continues. Then
nominal gains; then the sway test (wearer/rig moved gently while the tip
is watched against a fixed room reference). Deliberate Mount occlusion
mid-hold exercises freeze/re-anchor live. Every claim afterwards cites
`world_err_m` from the log, not impressions.

## 6. Deliberately out of scope

`V_base,E` and any velocity estimation (next slice, per the feedback-only
decision); torso targets; calibration beyond the identity assumption;
panel display changes beyond the columns appearing (the name-based reader
already surfaces them).
